import argparse
import os
import xml.etree.ElementTree as ET


def fix_lanelet_osm(input_path: str, output_path: str):
    # Resolve to absolute paths
    abs_input = os.path.abspath(input_path)
    abs_output = os.path.abspath(output_path)

    tree = ET.parse(abs_input)
    root = tree.getroot()

    for node in root.findall("node"):
        # 1. Convert lat/lon from scientific notation to standard floating point
        if "lat" in node.attrib:
            node.attrib["lat"] = f"{float(node.attrib['lat']):.15f}"
        if "lon" in node.attrib:
            node.attrib["lon"] = f"{float(node.attrib['lon']):.15f}"

        # 2. Add 'ele' tag if missing
        has_ele = any(
            tag.attrib.get("k") == "ele" for tag in node.findall("tag")
        )
        if not has_ele:
            ele_tag = ET.SubElement(node, "tag")
            ele_tag.attrib["k"] = "ele"
            ele_tag.attrib["v"] = "0.0"

    # Ensure output directory exists before writing
    os.makedirs(os.path.dirname(abs_output), exist_ok=True)
    tree.write(abs_output, encoding="utf-8", xml_declaration=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Fix scientific notation and missing elevation tags in Lanelet2 OSM files."
    )
    parser.add_argument("input_path", type=str, help="Path to input OSM file")
    parser.add_argument(
        "output_path", type=str, help="Path to save fixed OSM file"
    )

    args = parser.parse_args()
    fix_lanelet_osm(args.input_path, args.output_path)