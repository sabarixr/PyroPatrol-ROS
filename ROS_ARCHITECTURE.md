## ROS Architecture (generated)

This diagram and summary were generated from the repository's static analysis of ROS Python nodes and launch files.

- Source JSON: `ros_architecture_filtered.json` (only topics with at least one publisher and at least one subscriber are included)
- Graph: `ros_graph_filtered.dot` → rendered as `ros_graph_filtered.svg`

### How to view

Open the generated SVG file:

```bash
xdg-open ros_graph_filtered.svg
```

Or render the DOT yourself with Graphviz:

```bash
dot -Tsvg ros_graph_filtered.dot -o ros_graph_filtered.svg
```

### What this shows

- Nodes (boxes) and Topics (ovals).
- Edges from Node -> Topic indicate publishing.
- Edges from Topic -> Node indicate subscribing.
- Only topics with both publishers and subscribers are included, and only nodes connected to those topics.

### Quick notes

- This is a static analysis (text-search based). It can miss dynamic topic names, C/C++ nodes, runtime remappings, and nodes launched outside the analyzed launch files.
- Use `rqt_graph` at runtime for the accurate live topology.

### Files created

- `ros_architecture_filtered.json` — JSON structured report used to build the graph
- `ros_graph_filtered.dot` — DOT source used to render the graph
- `ros_graph_filtered.svg` — Rendered SVG graph (viewable in a browser or image viewer)

---

If you want, I can also embed a small, human-readable node/topic table in this file (auto-generated from the JSON). Would you like that appended?
