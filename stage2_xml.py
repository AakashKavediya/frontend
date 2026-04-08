import json
from pathlib import Path

import ifcopenshell
from ifcopenshell.util import element as element_utils
from ifcopenshell.util import placement as placement_utils
from ifcopenshell.util import representation as representation_utils

# Optional geometry support
try:
    import ifcopenshell.geom
    GEOM_AVAILABLE = True
except Exception:
    GEOM_AVAILABLE = False


IFC_PATH = r"Snowdon_Towers_Sample_Architectural.ifc"
OUTPUT_JSON = "ifc_extracted_data.json"

ifc = ifcopenshell.open(IFC_PATH)

# Geometry settings
geom_settings = None
if GEOM_AVAILABLE:
    geom_settings = ifcopenshell.geom.settings()
    geom_settings.set(geom_settings.USE_WORLD_COORDS, True)


def json_safe(value):
    if value is None:
        return None
    if isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    if hasattr(value, "is_a"):
        try:
            return {
                "id": value.id() if hasattr(value, "id") else None,
                "type": value.is_a(),
                "name": getattr(value, "Name", None),
            }
        except Exception:
            return str(value)
    return str(value)


def get_storey(element):
    try:
        container = element_utils.get_container(element)
        if container:
            return {
                "id": container.id(),
                "name": getattr(container, "Name", None),
                "type": container.is_a(),
            }
    except Exception:
        pass
    return None


def get_material(element):
    try:
        material = element_utils.get_material(element)
        if material:
            return {
                "id": material.id(),
                "name": getattr(material, "Name", None),
                "type": material.is_a(),
            }
    except Exception:
        pass
    return None


def get_properties(element):
    """
    Returns a flattened dictionary of all property sets + quantities where possible.
    """
    props = {}
    try:
        psets = element_utils.get_psets(element, psets_only=False, qtos=True)
        if psets:
            for set_name, set_data in psets.items():
                if isinstance(set_data, dict):
                    props[set_name] = {}
                    for k, v in set_data.items():
                        props[set_name][k] = json_safe(v)
                else:
                    props[set_name] = json_safe(set_data)
    except Exception:
        pass
    return props


def get_quantities(element):
    quantities = {}
    try:
        psets = element_utils.get_psets(element, qtos=True)
        for set_name, set_data in psets.items():
            if isinstance(set_data, dict) and set_name.lower().startswith("qto_"):
                quantities[set_name] = set_data
    except Exception:
        pass
    return quantities


def get_bounding_box(element):
    """
    Uses geometry when available. Returns min/max corners.
    """
    if not GEOM_AVAILABLE or geom_settings is None:
        return None

    try:
        shape = ifcopenshell.geom.create_shape(geom_settings, element)
        verts = shape.geometry.verts
        if not verts:
            return None

        xs = verts[0::3]
        ys = verts[1::3]
        zs = verts[2::3]

        return {
            "min": {"x": min(xs), "y": min(ys), "z": min(zs)},
            "max": {"x": max(xs), "y": max(ys), "z": max(zs)},
        }
    except Exception:
        return None


def _extract_point_from_cartesian_point(pt):
    try:
        coords = pt.Coordinates
        return {
            "x": float(coords[0]) if len(coords) > 0 else None,
            "y": float(coords[1]) if len(coords) > 1 else None,
            "z": float(coords[2]) if len(coords) > 2 else None,
        }
    except Exception:
        return None


def get_axis_points(element):
    """
    Tries to extract start/end points from Axis/Curve representations.
    Works best for linear MEP elements if the IFC contains axis geometry.
    """
    try:
        rep = getattr(element, "Representation", None)
        if not rep:
            return None, None

        for shape_rep in rep.Representations or []:
            ident = (getattr(shape_rep, "RepresentationIdentifier", "") or "").lower()
            if ident not in ("axis", "curve", "path"):
                continue

            items = getattr(shape_rep, "Items", None) or []
            for item in items:
                # IfcPolyline
                if item.is_a("IfcPolyline"):
                    pts = [_extract_point_from_cartesian_point(p) for p in item.Points]
                    pts = [p for p in pts if p is not None]
                    if len(pts) >= 2:
                        return pts[0], pts[-1]

                # IfcTrimmedCurve
                if item.is_a("IfcTrimmedCurve"):
                    base = item.BasisCurve
                    # Try to interpret trimmed curve endpoints from trim points
                    trim1 = None
                    trim2 = None
                    if getattr(item, "Trim1", None):
                        trim1 = item.Trim1[0] if len(item.Trim1) else None
                    if getattr(item, "Trim2", None):
                        trim2 = item.Trim2[0] if len(item.Trim2) else None

                    # Trim items can be ratios or points; handle point case
                    p1 = _extract_point_from_trim(trim1)
                    p2 = _extract_point_from_trim(trim2)
                    if p1 or p2:
                        return p1, p2

                # IfcIndexedPolyCurve
                if item.is_a("IfcIndexedPolyCurve"):
                    coords = item.Points.CoordList
                    pts = [{"x": float(c[0]), "y": float(c[1]), "z": float(c[2]) if len(c) > 2 else 0.0} for c in coords]
                    if len(pts) >= 2:
                        return pts[0], pts[-1]

    except Exception:
        pass

    return None, None


def _extract_point_from_trim(trim_item):
    """
    Best-effort extraction for curve trim point if it is a point-based trim.
    """
    try:
        if trim_item is None:
            return None
        if hasattr(trim_item, "is_a") and trim_item.is_a("IfcCartesianPoint"):
            return _extract_point_from_cartesian_point(trim_item)
    except Exception:
        pass
    return None


def get_diameter(element):
    """
    Tries multiple sources:
    1) Property sets
    2) Profile definitions
    3) Direct IFC attributes where available
    """
    # 1) From property sets
    try:
        psets = element_utils.get_psets(element, psets_only=False, qtos=True)
        for _, set_data in psets.items():
            if isinstance(set_data, dict):
                for key in ("Diameter", "NominalDiameter", "OverallDiameter", "PipeDiameter", "DuctDiameter", "Size"):
                    if key in set_data and set_data[key] not in (None, "", "None"):
                        try:
                            return float(set_data[key])
                        except Exception:
                            return set_data[key]
    except Exception:
        pass

    # 2) From representation/profile
    try:
        rep = getattr(element, "Representation", None)
        if rep:
            for shape_rep in rep.Representations or []:
                for item in getattr(shape_rep, "Items", []) or []:
                    if item.is_a("IfcSweptDiskSolid"):
                        try:
                            return float(item.Radius) * 2.0
                        except Exception:
                            pass
                    if item.is_a("IfcExtrudedAreaSolid"):
                        profile = item.SweptArea
                        if profile and profile.is_a("IfcCircleProfileDef"):
                            try:
                                return float(profile.Radius) * 2.0
                            except Exception:
                                pass
    except Exception:
        pass

    return None


def extract_element(element):
    start_point, end_point = get_axis_points(element)

    return {
        "ifc_internal_id": element.id(),
        "element_id": getattr(element, "GlobalId", None),
        "type": element.is_a(),
        "name": getattr(element, "Name", None),
        "description": getattr(element, "Description", None),
        "predefined_type": getattr(element, "PredefinedType", None),
        "object_type": getattr(element, "ObjectType", None),
        "storey": get_storey(element),
        "material": get_material(element),
        "bounding_box": get_bounding_box(element),
        "start_point": start_point,
        "end_point": end_point,
        "diameter": get_diameter(element),
        "properties": get_properties(element),
        "quantities": get_quantities(element),
    }


# You can use ifc.by_type("IfcProduct") for model elements,
# or loop through every entity with `for element in ifc:`.
results = []

for element in ifc.by_type("IfcProduct"):
    try:
        results.append(extract_element(element))
    except Exception as exc:
        results.append({
            "ifc_internal_id": element.id() if hasattr(element, "id") else None,
            "element_id": getattr(element, "GlobalId", None),
            "type": element.is_a() if hasattr(element, "is_a") else None,
            "error": str(exc),
        })

with open(OUTPUT_JSON, "w", encoding="utf-8") as f:
    json.dump(results, f, indent=2)

print(f"Saved JSON to: {Path(OUTPUT_JSON).resolve()}")
print(f"Total elements extracted: {len(results)}")

# Optional terminal preview
preview_rows = []
for row in results[:25]:
    preview_rows.append([
        row.get("element_id"),
        row.get("type"),
        row.get("name"),
        row.get("diameter"),
        row.get("start_point"),
        row.get("end_point"),
    ])

try:
    from tabulate import tabulate
    print(tabulate(preview_rows, headers=["element_id", "type", "name", "diameter", "start_point", "end_point"], tablefmt="grid"))
except Exception:
    for r in preview_rows:
        print(r)


# import xml.etree.ElementTree as ET
# import csv
# from tabulate import tabulate

# # Parse the XML file
# tree = ET.parse('hero.xml')
# root = tree.getroot()

# # Find all clashresult elements
# clash_results = root.findall('.//clashresult')

# # Prepare data for CSV
# data = []
# for clash in clash_results:
#     # Get basic clash info
#     name = clash.get('name', '')
#     guid = clash.get('guid', '')
#     status = clash.get('status', '')
#     distance = clash.get('distance', '')
    
#     # Get clashpoint
#     clashpoint_elem = clash.find('.//clashpoint/pos3f')
#     x = clashpoint_elem.get('x', '') if clashpoint_elem is not None else ''
#     y = clashpoint_elem.get('y', '') if clashpoint_elem is not None else ''
#     z = clashpoint_elem.get('z', '') if clashpoint_elem is not None else ''
    
#     # Get grid location
#     grid_location_elem = clash.find('gridlocation')
#     grid_location = grid_location_elem.text if grid_location_elem is not None else ''
    
#     # Get result status
#     result_status_elem = clash.find('resultstatus')
#     result_status = result_status_elem.text if result_status_elem is not None else ''
    
#     # Get created date
#     date_elem = clash.find('.//createddate/date')
#     year = date_elem.get('year', '') if date_elem is not None else ''
#     month = date_elem.get('month', '') if date_elem is not None else ''
#     day = date_elem.get('day', '') if date_elem is not None else ''
#     hour = date_elem.get('hour', '') if date_elem is not None else ''
#     minute = date_elem.get('minute', '') if date_elem is not None else ''
#     second = date_elem.get('second', '') if date_elem is not None else ''
#     created_date = f"{year}-{month}-{day} {hour}:{minute}:{second}" if year else ''
    
#     # Get clash objects
#     clash_objects = clash.findall('.//clashobject')
#     obj1_element_id = ''
#     obj1_item_name = ''
#     obj1_item_type = ''
#     obj1_layer = ''
#     obj2_element_id = ''
#     obj2_item_name = ''
#     obj2_item_type = ''
#     obj2_layer = ''
    
#     for idx, obj in enumerate(clash_objects):
#         # Get element ID
#         elem_id_elem = obj.find(".//objectattribute[name='Element ID']/value")
#         elem_id = elem_id_elem.text if elem_id_elem is not None else ''
        
#         # Get layer
#         layer_elem = obj.find('layer')
#         layer = layer_elem.text if layer_elem is not None else ''
        
#         # Get item name and type from smarttags
#         item_name = ''
#         item_type = ''
#         smarttags = obj.findall('.//smarttag')
#         for st in smarttags:
#             name_elem = st.find('name')
#             value_elem = st.find('value')
#             if name_elem is not None and value_elem is not None:
#                 if name_elem.text == 'Item Name':
#                     item_name = value_elem.text
#                 elif name_elem.text == 'Item Type':
#                     item_type = value_elem.text
        
#         if idx == 0:
#             obj1_element_id = elem_id
#             obj1_item_name = item_name
#             obj1_item_type = item_type
#             obj1_layer = layer
#         elif idx == 1:
#             obj2_element_id = elem_id
#             obj2_item_name = item_name
#             obj2_item_type = item_type
#             obj2_layer = layer
    
#     data.append([
#         name, guid, status, distance, x, y, z, grid_location, result_status,
#         created_date, obj1_element_id, obj1_item_name, obj1_item_type, obj1_layer,
#         obj2_element_id, obj2_item_name, obj2_item_type, obj2_layer
#     ])

# # Define headers
# headers = [
#     'Clash Name', 'GUID', 'Status', 'Distance (m)', 'Clash X', 'Clash Y', 'Clash Z',
#     'Grid Location', 'Result Status', 'Created Date',
#     'Object 1 - Element ID', 'Object 1 - Item Name', 'Object 1 - Item Type', 'Object 1 - Layer',
#     'Object 2 - Element ID', 'Object 2 - Item Name', 'Object 2 - Item Type', 'Object 2 - Layer'
# ]

# # Write to CSV file
# with open('clash_results.csv', 'w', newline='', encoding='utf-8') as csvfile:
#     writer = csv.writer(csvfile)
#     writer.writerow(headers)
#     writer.writerows(data)

# print(f"Successfully exported {len(data)} clash records to 'clash_results.csv'")
# print("\n" + "="*120)
# print("FIRST 20 ROWS OF CLASH RESULTS")
# print("="*120 + "\n")

# # Display first 20 rows in tabular format
# print(tabulate(data[:20], headers=headers, tablefmt='grid', maxcolwidths=25))

# # Also print summary statistics
# print("\n" + "="*120)
# print("SUMMARY STATISTICS")
# print("="*120)

# # Count by status
# status_count = {}
# for row in data:
#     status = row[2]  # Status column
#     status_count[status] = status_count.get(status, 0) + 1

# print("\nClash Status Distribution:")
# for status, count in status_count.items():
#     print(f"  {status}: {count}")

# # Count by grid location
# grid_count = {}
# for row in data:
#     grid = row[7]  # Grid Location column
#     if grid:
#         grid_count[grid] = grid_count.get(grid, 0) + 1

# print("\nTop 10 Grid Locations with Most Clashes:")
# for grid, count in sorted(grid_count.items(), key=lambda x: x[1], reverse=True)[:10]:
#     print(f"  {grid}: {count}")

# # Count by item type combinations
# item_type_pairs = {}
# for row in data:
#     obj1_type = row[12]  # Object 1 - Item Type
#     obj2_type = row[16]  # Object 2 - Item Type
#     pair = f"{obj1_type} vs {obj2_type}"
#     item_type_pairs[pair] = item_type_pairs.get(pair, 0) + 1

# print("\nTop 10 Item Type Conflict Pairs:")
# for pair, count in sorted(item_type_pairs.items(), key=lambda x: x[1], reverse=True)[:10]:
#     print(f"  {pair}: {count}")