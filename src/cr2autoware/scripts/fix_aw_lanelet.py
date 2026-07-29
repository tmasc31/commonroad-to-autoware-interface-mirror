import argparse
import os
import xml.etree.ElementTree as ET


def set_or_update_tag(element: ET.Element, key: str, value: str):
    """Helper to update an existing tag or append a new one at the bottom."""
    for tag in element.findall("tag"):
        if tag.attrib.get("k") == key:
            tag.attrib["v"] = value
            return
    # If not found, append to the end
    ET.SubElement(element, "tag", {"k": key, "v": value})


def fix_lanelet_osm(input_path: str, output_path: str):
    abs_input = os.path.abspath(input_path)
    abs_output = os.path.abspath(output_path)

    tree = ET.parse(abs_input)
    root = tree.getroot()

    # Process nodes (lat/lon float formatting and default elevation)
    for node in root.findall("node"):
        if "lat" in node.attrib:
            node.attrib["lat"] = f"{float(node.attrib['lat']):.15f}"
        if "lon" in node.attrib:
            node.attrib["lon"] = f"{float(node.attrib['lon']):.15f}"

        has_ele = any(
            tag.attrib.get("k") == "ele" for tag in node.findall("tag")
        )
        if not has_ele:
            ET.SubElement(node, "tag", {"k": "ele", "v": "0.0"})

    # Add line tags to all ways
    for way in root.findall("way"):
        set_or_update_tag(way, "type", "line_thin")
        set_or_update_tag(way, "subtype", "solid")
        set_or_update_tag(way, "width", "0.200")

    # Ensure subtype=road on all lanelet relations
    for relation in root.findall("relation"):
        is_lanelet = any(
            tag.attrib.get("k") == "type" and tag.attrib.get("v") == "lanelet"
            for tag in relation.findall("tag")
        )
        if is_lanelet:
            set_or_update_tag(relation, "subtype", "road")

    os.makedirs(os.path.dirname(abs_output), exist_ok=True)
    tree.write(abs_output, encoding="utf-8", xml_declaration=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Fix scientific notation, missing elevation, way attributes, and lanelet relation tags in Lanelet2 OSM files."
    )
    parser.add_argument("input_path", type=str, help="Path to input OSM file")
    parser.add_argument(
        "output_path", type=str, help="Path to save fixed OSM file"
    )

    args = parser.parse_args()
    fix_lanelet_osm(args.input_path, args.output_path)