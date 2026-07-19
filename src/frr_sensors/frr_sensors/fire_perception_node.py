#!/usr/bin/env python3
import json
from pathlib import Path

import joblib
import numpy as np
import rclpy
import torch
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String

from .fire_model import MultiSectorCNN

class FirePerceptionNode(Node):
    def __init__(self):
        super().__init__('fire_perception_node')

        # ROS Interfaces
        self.telemetry_sub = self.create_subscription(String, '/esp32_telemetry', self.telemetry_callback, 10)
        self.perception_pub = self.create_publisher(String, '/mission/fire_perception', 10)

        # Load AI models from the installed package share.
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        model_dir = Path(get_package_share_directory('frr_sensors')) / 'models'

        self.get_logger().info('Loading MSCNN model and scaler...')
        try:
            self.scaler = joblib.load(model_dir / 'mscnn_scaler.pkl')
            self.config = joblib.load(model_dir / 'mscnn_config.pkl')
            self.classes = np.load(model_dir / 'mscnn_classes.npy', allow_pickle=True)

            self.model = MultiSectorCNN(
                self.config['n_channels'],
                self.config['n_classes'],
                self.config['d_model'],
                self.config['n_heads']
            ).to(self.device)
            self.model.load_state_dict(torch.load(model_dir / 'mscnn_direction.pt', map_location=self.device, weights_only=True))
            self.model.eval()
            self.get_logger().info('MSCNN model loaded successfully.')
        except Exception as e:
            raise RuntimeError(f'Failed to load AI model from {model_dir}: {e}') from e

        # Buffers for multi-sector scan (LEFT, FRONT, RIGHT)
        self.N_TIMESTEPS = 15
        self.SECTOR_ORDER = ["LEFT", "FRONT", "RIGHT"]

        self.current_sector = None
        self.current_buffer = []
        self.sector_buffers = {"LEFT": None, "FRONT": None, "RIGHT": None}

    def extract_features(self, data):
        """Extract the exactly 10 SENSOR_COLS required by the CNN."""
        return [
            float(data.get('servo_angle', 90)),
            float(data.get('mq2', 0)),
            float(data.get('mq5', 0)),
            float(data.get('temp', 25.0)),
            float(data.get('flame_left', 1)),
            float(data.get('flame_center', 1)),
            float(data.get('flame_right', 1)),
            float(data.get('ax', 0)),
            float(data.get('ay', 0)),
            float(data.get('az', 1)),
        ]

    def telemetry_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            msg_type = data.get('type')

            if msg_type == 'scan_sample':
                sector = data.get('sector', '').upper()
                if sector not in self.SECTOR_ORDER:
                    return

                # If sector changed, process the previous sector buffer
                if sector != self.current_sector:
                    if self.current_sector is not None and len(self.current_buffer) > 0:
                        self.save_sector_buffer(self.current_sector, self.current_buffer)
                    self.current_sector = sector
                    self.current_buffer = []

                self.current_buffer.append(self.extract_features(data))

            elif msg_type == 'scan_complete':
                # Finalize the last sector
                if self.current_sector is not None and len(self.current_buffer) > 0:
                    self.save_sector_buffer(self.current_sector, self.current_buffer)
                self.current_sector = None
                self.current_buffer = []

                # Check if we have all 3 sectors
                if all(self.sector_buffers[s] is not None for s in self.SECTOR_ORDER):
                    self.run_inference()
                else:
                    self.get_logger().warn("Incomplete scan received (missing sectors).")

                # Clear buffers for the next scan sweep
                self.sector_buffers = {"LEFT": None, "FRONT": None, "RIGHT": None}

        except Exception as e:
            self.get_logger().error(f"Error in telemetry callback: {e}")

    def save_sector_buffer(self, sector, buffer):
        """Format the buffer to exactly 15 timesteps and transpose to (Channels, Timesteps)."""
        readings = np.array(buffer, dtype=np.float32)  # (N, C)
        n = len(readings)

        if n >= self.N_TIMESTEPS:
            readings = readings[:self.N_TIMESTEPS]
        else:
            # Pad with the last reading
            pad = np.tile(readings[-1:], (self.N_TIMESTEPS - n, 1))
            readings = np.vstack([readings, pad])

        self.sector_buffers[sector] = readings.T  # (C, T)

    def run_inference(self):
        """Run the Dual-Head MSCNN on the collected 3-sector scan."""
        # Stack sectors in order: (3, C, T)
        tensor = np.stack([self.sector_buffers[s] for s in self.SECTOR_ORDER], axis=0)

        # Flatten for scaler: scaler expects (3*T, C)
        C, T = tensor.shape[1], tensor.shape[2]
        flat = tensor.transpose(0, 2, 1).reshape(-1, C)
        flat_scaled = self.scaler.transform(flat).astype(np.float32)

        # Unflatten back to (1, 3, C, T)
        tensor_scaled = flat_scaled.reshape(1, 3, T, C).transpose(0, 1, 3, 2)

        x_tensor = torch.tensor(tensor_scaled, dtype=torch.float32).to(self.device)

        with torch.no_grad():
            out_dir, out_sev = self.model(x_tensor)

            # Get direction class
            pred_idx = out_dir.argmax(1).item()
            pred_dir = self.classes[pred_idx]

            # Get severity score
            pred_sev = out_sev.item()

            # (Optional) Get confidence softmax for direction
            probs = torch.softmax(out_dir, dim=1)[0]
            confidence = probs[pred_idx].item()

        self.get_logger().info(
            f"AI Prediction | Dir: {pred_dir.upper()} ({confidence*100:.1f}%) | Severity: {pred_sev:.3f}"
        )

        # Publish to mission controller
        msg = String()
        msg.data = json.dumps({
            'direction': pred_dir.lower(),
            'confidence': confidence,
            'severity': pred_sev
        })
        self.perception_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FirePerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
