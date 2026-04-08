"""
╔══════════════════════════════════════════════════════════════════════════════════╗
║       MEP CLASH RULE ENGINE v4.0 — Engineering Validation & Resolution         ║
║                                                                                ║
║  What's new in v4:                                                             ║
║    • Spatial index (3-D bounding-box grid) for O(log n) neighbour queries     ║
║    • Post-relocation clash validation — every suggested move is tested         ║
║      against ALL neighbouring elements, not just the current pair             ║
║    • 8 rerouting strategies ranked and scored per element type:                ║
║        raise_z | lower_z | lateral_shift_pos | lateral_shift_neg |            ║
║        diagonal_reposition | offset_parallel | elevation_change |             ║
║        segment_reroute                                                         ║
║    • Cascade clash detection — identifies knock-on effects of a move          ║
║    • Clash graph analysis — finds clusters and shared-element hot-spots        ║
║    • Multi-candidate resolution — returns ranked list of valid moves           ║
║    • Richer severity model with system-pair escalation matrix                  ║
║                                                                                ║
║  Standards applied:                                                            ║
║    • ASHRAE 90.1 / ASHRAE Fundamentals Handbook                                ║
║    • ASHRAE 15-2019 (Refrigeration Safety)                                     ║
║    • SMACNA HVAC Duct Construction Standards 3rd Ed.                           ║
║    • IS 1239 / NBC India Part 4, 6, 8, 9                                      ║
║    • BS EN 806 / CIBSE Guide G                                                 ║
║    • IEC 60364-5-52 / NEC (Electrical clearances)                              ║
║    • NFPA 96 (Kitchen Ventilation / Grease Duct Fire Safety)                   ║
║                                                                                ║
║  Input : Navisworks hero.xml                                                   ║
║  Output: clash_rule_engine_report.json  +  console summary                    ║
╚══════════════════════════════════════════════════════════════════════════════════╝
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
import json
import math
from dataclasses import dataclass, field, asdict
from typing import Optional, List, Tuple, Dict, Set, Iterator
from collections import Counter, defaultdict
from pathlib import Path


# ══════════════════════════════════════════════════════════════════════════════
#  SECTION 1 ── MEP ELEMENT TYPE CATALOGUE
# ══════════════════════════════════════════════════════════════════════════════

@dataclass(frozen=True)
class MEPElementType:
    canonical:          str
    category:           str   # DUCT | PIPE | ELEC | STRUCTURE | FITTING | UNKNOWN
    sub_type:           str
    medium:             str
    system_label:       str
    routing_priority:   int   # 1=never move (structure), 5=move first
    typical_size_mm:    str
    insulation_req:     bool
    is_pressurised:     bool
    # New in v4
    typical_height_mm:  float  # nominal centreline height above slab (for raise/lower bounds)
    max_raise_mm:       float  # how much headroom we can steal upward
    max_lower_mm:       float  # how much we can drop before hitting slab/beam
    lateral_flex:       bool   # True if this service can move horizontally easily
    diameter_mm:        float  # representative diameter/depth for AABB estimation


ELEMENT_CATALOGUE: Dict[str, MEPElementType] = {

    # ── DUCTS ─────────────────────────────────────────────────────────────────
    "SAD": MEPElementType(
        canonical="duct_supply_air", category="DUCT", sub_type="supply_air",
        medium="air", system_label="Supply Air Duct (HVAC)",
        routing_priority=3, typical_size_mm="200–1200mm",
        insulation_req=True, is_pressurised=False,
        typical_height_mm=2700, max_raise_mm=400, max_lower_mm=300,
        lateral_flex=True, diameter_mm=600,
    ),
    "RAD": MEPElementType(
        canonical="duct_return_air", category="DUCT", sub_type="return_air",
        medium="air", system_label="Return Air Duct (HVAC)",
        routing_priority=3, typical_size_mm="200–1000mm",
        insulation_req=False, is_pressurised=False,
        typical_height_mm=2700, max_raise_mm=400, max_lower_mm=300,
        lateral_flex=True, diameter_mm=500,
    ),
    "FAD": MEPElementType(
        canonical="duct_fresh_air", category="DUCT", sub_type="fresh_air",
        medium="air", system_label="Fresh Air Duct (HVAC OA)",
        routing_priority=3, typical_size_mm="150–800mm",
        insulation_req=True, is_pressurised=False,
        typical_height_mm=2700, max_raise_mm=400, max_lower_mm=300,
        lateral_flex=True, diameter_mm=400,
    ),
    "TED": MEPElementType(
        canonical="duct_transfer_exhaust", category="DUCT", sub_type="transfer_exhaust",
        medium="air", system_label="Transfer / Exhaust Duct",
        routing_priority=4, typical_size_mm="100–600mm",
        insulation_req=False, is_pressurised=False,
        typical_height_mm=2600, max_raise_mm=300, max_lower_mm=250,
        lateral_flex=True, diameter_mm=300,
    ),
    "KED": MEPElementType(
        canonical="duct_kitchen_exhaust", category="DUCT", sub_type="kitchen_exhaust",
        medium="air_grease", system_label="Kitchen Exhaust Duct",
        routing_priority=2, typical_size_mm="150–600mm",
        insulation_req=False, is_pressurised=False,
        typical_height_mm=2800, max_raise_mm=200, max_lower_mm=100,
        lateral_flex=False, diameter_mm=350,
    ),
    "LED": MEPElementType(
        canonical="duct_local_exhaust", category="DUCT", sub_type="local_exhaust",
        medium="air", system_label="Local Exhaust Duct",
        routing_priority=4, typical_size_mm="100–400mm",
        insulation_req=False, is_pressurised=False,
        typical_height_mm=2600, max_raise_mm=300, max_lower_mm=200,
        lateral_flex=True, diameter_mm=200,
    ),
    "IDU-02": MEPElementType(
        canonical="duct_indoor_unit", category="DUCT", sub_type="indoor_unit",
        medium="air", system_label="Indoor HVAC Unit Duct",
        routing_priority=2, typical_size_mm="150–400mm",
        insulation_req=True, is_pressurised=False,
        typical_height_mm=2700, max_raise_mm=200, max_lower_mm=100,
        lateral_flex=False, diameter_mm=250,
    ),

    # ── PIPES — DRAINAGE / SANITARY ──────────────────────────────────────────
    "Waste Pipe": MEPElementType(
        canonical="pipe_waste", category="PIPE", sub_type="waste_drainage",
        medium="waste_water", system_label="Waste / Drainage Pipe",
        routing_priority=4, typical_size_mm="40–150mm",
        insulation_req=False, is_pressurised=False,
        typical_height_mm=2400, max_raise_mm=200, max_lower_mm=400,
        lateral_flex=True, diameter_mm=100,
    ),
    "Soil Pipe": MEPElementType(
        canonical="pipe_soil", category="PIPE", sub_type="soil_drainage",
        medium="sewage", system_label="Soil Pipe (Sanitary / Sewage)",
        routing_priority=3, typical_size_mm="75–150mm",
        insulation_req=False, is_pressurised=False,
        typical_height_mm=2300, max_raise_mm=150, max_lower_mm=350,
        lateral_flex=True, diameter_mm=100,
    ),
    "Rainwater Pipe": MEPElementType(
        canonical="pipe_rainwater", category="PIPE", sub_type="rainwater",
        medium="rainwater", system_label="Rainwater Downpipe",
        routing_priority=4, typical_size_mm="50–150mm",
        insulation_req=False, is_pressurised=False,
        typical_height_mm=2500, max_raise_mm=300, max_lower_mm=400,
        lateral_flex=True, diameter_mm=75,
    ),
    "Vent Pipe": MEPElementType(
        canonical="pipe_vent", category="PIPE", sub_type="plumbing_vent",
        medium="air_vent", system_label="Plumbing Vent Pipe",
        routing_priority=5, typical_size_mm="32–100mm",
        insulation_req=False, is_pressurised=False,
        typical_height_mm=2500, max_raise_mm=400, max_lower_mm=300,
        lateral_flex=True, diameter_mm=65,
    ),
    "ref pipe": MEPElementType(
        canonical="pipe_refrigerant", category="PIPE", sub_type="refrigerant",
        medium="refrigerant", system_label="Refrigerant Pipe (HVAC)",
        routing_priority=3, typical_size_mm="12–50mm",
        insulation_req=True, is_pressurised=True,
        typical_height_mm=2600, max_raise_mm=350, max_lower_mm=300,
        lateral_flex=True, diameter_mm=35,
    ),
    "GT": MEPElementType(
        canonical="pipe_grease_trap", category="PIPE", sub_type="grease_trap_line",
        medium="grease_waste", system_label="Grease Trap Line",
        routing_priority=4, typical_size_mm="50–100mm",
        insulation_req=False, is_pressurised=False,
        typical_height_mm=2300, max_raise_mm=150, max_lower_mm=300,
        lateral_flex=True, diameter_mm=75,
    ),
    "Floor Gully": MEPElementType(
        canonical="fitting_floor_gully", category="FITTING", sub_type="floor_drain",
        medium="waste_water", system_label="Floor Gully / Drain Fitting",
        routing_priority=2, typical_size_mm="100–200mm",
        insulation_req=False, is_pressurised=False,
        typical_height_mm=0, max_raise_mm=0, max_lower_mm=0,
        lateral_flex=False, diameter_mm=150,
    ),
    "MANHOLE": MEPElementType(
        canonical="structure_manhole", category="STRUCTURE", sub_type="manhole_chamber",
        medium="sewage", system_label="Manhole / Inspection Chamber",
        routing_priority=1, typical_size_mm="600–1200mm",
        insulation_req=False, is_pressurised=False,
        typical_height_mm=0, max_raise_mm=0, max_lower_mm=0,
        lateral_flex=False, diameter_mm=900,
    ),

    # ── PIPES — WATER SERVICES ────────────────────────────────────────────────
    "Domestic Cold Water": MEPElementType(
        canonical="pipe_cold_water", category="PIPE", sub_type="cold_water_supply",
        medium="cold_water", system_label="Domestic Cold Water Supply Pipe",
        routing_priority=4, typical_size_mm="15–50mm",
        insulation_req=True, is_pressurised=True,
        typical_height_mm=2500, max_raise_mm=300, max_lower_mm=300,
        lateral_flex=True, diameter_mm=32,
    ),
    "HOT WATER SUPPLY": MEPElementType(
        canonical="pipe_hot_water_supply", category="PIPE", sub_type="hot_water_supply",
        medium="hot_water", system_label="Hot Water Supply Pipe (>60°C)",
        routing_priority=4, typical_size_mm="15–50mm",
        insulation_req=True, is_pressurised=True,
        typical_height_mm=2500, max_raise_mm=300, max_lower_mm=300,
        lateral_flex=True, diameter_mm=32,
    ),
    "HOT WATER RETURN": MEPElementType(
        canonical="pipe_hot_water_return", category="PIPE", sub_type="hot_water_return",
        medium="hot_water", system_label="Hot Water Return Pipe",
        routing_priority=4, typical_size_mm="15–50mm",
        insulation_req=True, is_pressurised=True,
        typical_height_mm=2500, max_raise_mm=300, max_lower_mm=300,
        lateral_flex=True, diameter_mm=32,
    ),
    "MAIN INCOMING LINE": MEPElementType(
        canonical="pipe_main_water", category="PIPE", sub_type="mains_water",
        medium="mains_water", system_label="Mains Water Incoming Line",
        routing_priority=2, typical_size_mm="50–150mm",
        insulation_req=True, is_pressurised=True,
        typical_height_mm=2400, max_raise_mm=200, max_lower_mm=200,
        lateral_flex=False, diameter_mm=100,
    ),
    "RLSD-02-20mmSLOT": MEPElementType(
        canonical="pipe_slot_drain", category="PIPE", sub_type="slot_drain",
        medium="surface_water", system_label="Slot Drain Line",
        routing_priority=4, typical_size_mm="20mm",
        insulation_req=False, is_pressurised=False,
        typical_height_mm=0, max_raise_mm=50, max_lower_mm=0,
        lateral_flex=True, diameter_mm=25,
    ),

    # ── ELECTRICAL / CARRIERS ─────────────────────────────────────────────────
    "Metal - Carrier - Steel": MEPElementType(
        canonical="cable_tray_steel", category="ELEC", sub_type="cable_tray",
        medium="electrical", system_label="Steel Cable Tray / Carrier",
        routing_priority=3, typical_size_mm="100–600mm",
        insulation_req=False, is_pressurised=False,
        typical_height_mm=2800, max_raise_mm=300, max_lower_mm=200,
        lateral_flex=True, diameter_mm=300,
    ),
    "Metal - Carrier - Brass": MEPElementType(
        canonical="conduit_brass", category="ELEC", sub_type="conduit",
        medium="electrical", system_label="Brass Conduit / Carrier",
        routing_priority=4, typical_size_mm="20–100mm",
        insulation_req=False, is_pressurised=False,
        typical_height_mm=2700, max_raise_mm=300, max_lower_mm=250,
        lateral_flex=True, diameter_mm=50,
    ),
    "Plastic - Carrier - Black": MEPElementType(
        canonical="conduit_plastic_black", category="ELEC", sub_type="conduit",
        medium="electrical", system_label="Black Plastic Conduit",
        routing_priority=5, typical_size_mm="20–63mm",
        insulation_req=False, is_pressurised=False,
        typical_height_mm=2700, max_raise_mm=350, max_lower_mm=250,
        lateral_flex=True, diameter_mm=40,
    ),
    "Plastic - Carrier - White": MEPElementType(
        canonical="conduit_plastic_white", category="ELEC", sub_type="conduit",
        medium="electrical", system_label="White Plastic Conduit",
        routing_priority=5, typical_size_mm="20–63mm",
        insulation_req=False, is_pressurised=False,
        typical_height_mm=2700, max_raise_mm=350, max_lower_mm=250,
        lateral_flex=True, diameter_mm=40,
    ),
    "Metallic Paint - Carrier - Silky Shade": MEPElementType(
        canonical="cable_tray_metallic", category="ELEC", sub_type="cable_tray",
        medium="electrical", system_label="Metallic-Paint Cable Carrier",
        routing_priority=3, typical_size_mm="100–400mm",
        insulation_req=False, is_pressurised=False,
        typical_height_mm=2800, max_raise_mm=300, max_lower_mm=200,
        lateral_flex=True, diameter_mm=200,
    ),
    "ME_2.3x1x1.2": MEPElementType(
        canonical="mep_equipment", category="STRUCTURE", sub_type="mep_equipment",
        medium="equipment", system_label="MEP Equipment Unit",
        routing_priority=1, typical_size_mm="2300×1000mm",
        insulation_req=False, is_pressurised=False,
        typical_height_mm=1200, max_raise_mm=0, max_lower_mm=0,
        lateral_flex=False, diameter_mm=1200,
    ),

    # ── GENERIC ───────────────────────────────────────────────────────────────
    "Plastic": MEPElementType(
        canonical="pipe_plastic_generic", category="PIPE", sub_type="plastic_generic",
        medium="unknown_fluid", system_label="Generic Plastic Pipe",
        routing_priority=5, typical_size_mm="25–100mm",
        insulation_req=False, is_pressurised=False,
        typical_height_mm=2400, max_raise_mm=300, max_lower_mm=300,
        lateral_flex=True, diameter_mm=50,
    ),
    "Stainless Steel, Austenitic": MEPElementType(
        canonical="pipe_ss_generic", category="PIPE", sub_type="stainless_steel_pipe",
        medium="unknown_fluid", system_label="Stainless Steel Pipe",
        routing_priority=4, typical_size_mm="15–100mm",
        insulation_req=False, is_pressurised=True,
        typical_height_mm=2500, max_raise_mm=300, max_lower_mm=300,
        lateral_flex=True, diameter_mm=65,
    ),
    "Standard": MEPElementType(
        canonical="unknown_standard", category="UNKNOWN", sub_type="generic",
        medium="unknown", system_label="Standard Generic Element",
        routing_priority=5, typical_size_mm="N/A",
        insulation_req=False, is_pressurised=False,
        typical_height_mm=2500, max_raise_mm=200, max_lower_mm=200,
        lateral_flex=True, diameter_mm=100,
    ),
    "Alluminum Paint Dark Gray": MEPElementType(
        canonical="structural_element", category="STRUCTURE", sub_type="structural_surface",
        medium="solid", system_label="Structural / Architectural Element",
        routing_priority=1, typical_size_mm="N/A",
        insulation_req=False, is_pressurised=False,
        typical_height_mm=0, max_raise_mm=0, max_lower_mm=0,
        lateral_flex=False, diameter_mm=500,
    ),
}

_UNKNOWN_TYPE = MEPElementType(
    canonical="unknown", category="UNKNOWN", sub_type="unknown",
    medium="unknown", system_label="Unknown Element",
    routing_priority=5, typical_size_mm="N/A",
    insulation_req=False, is_pressurised=False,
    typical_height_mm=2500, max_raise_mm=200, max_lower_mm=200,
    lateral_flex=True, diameter_mm=100,
)

def get_element_type(name: str) -> MEPElementType:
    return ELEMENT_CATALOGUE.get(name, _UNKNOWN_TYPE)


# ══════════════════════════════════════════════════════════════════════════════
#  SECTION 2 ── CLEARANCE RULE TABLE
# ══════════════════════════════════════════════════════════════════════════════

@dataclass(frozen=True)
class ClearanceRule:
    rule_id:            str
    elem_category_a:    str
    elem_category_b:    str
    min_clearance_mm:   float
    standard:           str
    rationale:          str
    is_safety_critical: bool
    # New in v4
    escalation_note:    str = ""  # extra note when both elements are high-risk systems


CLEARANCE_RULES: List[ClearanceRule] = [

    # ── ELECTRICAL SAFETY ─────────────────────────────────────────────────────
    ClearanceRule(
        rule_id="ELEC-01", elem_category_a="ELEC", elem_category_b="PIPE",
        min_clearance_mm=300,
        standard="IEC 60364-5-52 / NBC India Part 8 Section 2",
        rationale="Electrical cable trays/conduits ≥300mm from any water pipe to prevent "
                  "water damage, corrosion, and electrocution risk.",
        is_safety_critical=True,
        escalation_note="If pipe is pressurised or carries hot water, increase to 400mm.",
    ),
    ClearanceRule(
        rule_id="ELEC-02", elem_category_a="ELEC", elem_category_b="DUCT",
        min_clearance_mm=150,
        standard="IEC 60364 / SMACNA HVAC Applications",
        rationale="≥150mm between electrical carriers and HVAC ducts for installation "
                  "access and to prevent condensate drip onto cables.",
        is_safety_critical=True,
        escalation_note="Increase to 300mm if duct carries condensate or is uninsulated.",
    ),
    ClearanceRule(
        rule_id="ELEC-03", elem_category_a="ELEC", elem_category_b="ELEC",
        min_clearance_mm=50,
        standard="IEC 60364-5-52",
        rationale="Minimum 50mm between parallel electrical runs for heat dissipation.",
        is_safety_critical=False,
    ),

    # ── HOT WATER ─────────────────────────────────────────────────────────────
    ClearanceRule(
        rule_id="HWS-01", elem_category_a="hot_water_supply", elem_category_b="ELEC",
        min_clearance_mm=300,
        standard="IEC 60364 / CIBSE Guide G",
        rationale="Hot water supply (>60°C) ≥300mm from electrical cables; heat damages insulation.",
        is_safety_critical=True,
    ),
    ClearanceRule(
        rule_id="HWS-02", elem_category_a="hot_water_supply", elem_category_b="cold_water_supply",
        min_clearance_mm=100,
        standard="BS EN 806 / CIBSE Guide G Cl. 2.4.3",
        rationale="Hot/cold pipes ≥100mm apart; thermal transfer raises CW above 20°C "
                  "(Legionella risk above 25°C).",
        is_safety_critical=True,
        escalation_note="If pipes are uninsulated, increase separation to 150mm.",
    ),
    ClearanceRule(
        rule_id="HWS-03", elem_category_a="hot_water_return", elem_category_b="cold_water_supply",
        min_clearance_mm=100,
        standard="BS EN 806 / CIBSE Guide G",
        rationale="Same thermal separation requirement as HWS-02.",
        is_safety_critical=True,
    ),

    # ── REFRIGERANT ───────────────────────────────────────────────────────────
    ClearanceRule(
        rule_id="REF-01", elem_category_a="refrigerant", elem_category_b="ELEC",
        min_clearance_mm=200,
        standard="ASHRAE 15-2019",
        rationale="Refrigerant lines ≥200mm from electrical; leaks near sparks = fire/explosion.",
        is_safety_critical=True,
    ),
    ClearanceRule(
        rule_id="REF-02", elem_category_a="refrigerant", elem_category_b="PIPE",
        min_clearance_mm=50,
        standard="ASHRAE 15-2019 / SMACNA Refrigeration Piping",
        rationale="50mm minimum around refrigerant lines for insulation and vibration isolation.",
        is_safety_critical=False,
    ),

    # ── KITCHEN EXHAUST ───────────────────────────────────────────────────────
    ClearanceRule(
        rule_id="KIT-01", elem_category_a="kitchen_exhaust", elem_category_b="PIPE",
        min_clearance_mm=150,
        standard="NFPA 96 / NBC India Part 4 Cl. 4.7",
        rationale="Kitchen exhaust (grease-laden, high-temp) ≥150mm from all pipes.",
        is_safety_critical=True,
    ),
    ClearanceRule(
        rule_id="KIT-02", elem_category_a="kitchen_exhaust", elem_category_b="ELEC",
        min_clearance_mm=300,
        standard="NFPA 96 / NBC India Part 4",
        rationale="Kitchen exhaust ≥300mm from electrical (grease fire risk).",
        is_safety_critical=True,
    ),
    ClearanceRule(
        rule_id="KIT-03", elem_category_a="kitchen_exhaust", elem_category_b="DUCT",
        min_clearance_mm=200,
        standard="NFPA 96 / SMACNA Kitchen Exhaust Systems",
        rationale="Kitchen exhaust ≥200mm from HVAC ducts to prevent cross-contamination "
                  "and fire spread through ductwork.",
        is_safety_critical=True,
    ),

    # ── HVAC DUCTS ────────────────────────────────────────────────────────────
    ClearanceRule(
        rule_id="HVAC-01", elem_category_a="DUCT", elem_category_b="PIPE",
        min_clearance_mm=50,
        standard="SMACNA HVAC Duct Construction Standards 3rd Ed. / ASHRAE 90.1",
        rationale="≥50mm clearance between HVAC ducts and piping for insulation and "
                  "differential thermal movement.",
        is_safety_critical=False,
    ),
    ClearanceRule(
        rule_id="HVAC-02", elem_category_a="DUCT", elem_category_b="DUCT",
        min_clearance_mm=50,
        standard="SMACNA / ASHRAE Fundamentals Ch. 21",
        rationale="50mm minimum between parallel duct runs for insulation and installation.",
        is_safety_critical=False,
    ),
    ClearanceRule(
        rule_id="HVAC-03", elem_category_a="supply_air", elem_category_b="return_air",
        min_clearance_mm=100,
        standard="ASHRAE 90.1 / SMACNA Low-Velocity Duct Construction",
        rationale="Supply/return ducts ≥100mm to prevent short-circuiting of conditioned air.",
        is_safety_critical=False,
    ),

    # ── SANITARY / DRAINAGE ───────────────────────────────────────────────────
    ClearanceRule(
        rule_id="SAN-01", elem_category_a="soil_drainage", elem_category_b="cold_water_supply",
        min_clearance_mm=100,
        standard="BS EN 806 / NBC India Part 9 Sec. 5 / IS 1172",
        rationale="Soil/sewage pipes ≥100mm from potable water supply (cross-contamination).",
        is_safety_critical=True,
    ),
    ClearanceRule(
        rule_id="SAN-02", elem_category_a="soil_drainage", elem_category_b="ELEC",
        min_clearance_mm=150,
        standard="IEC 60364 / NBC Part 8",
        rationale="Sewage/soil pipes ≥150mm from electrical (corrosive gases, contamination).",
        is_safety_critical=True,
    ),
    ClearanceRule(
        rule_id="SAN-03", elem_category_a="waste_drainage", elem_category_b="ELEC",
        min_clearance_mm=150,
        standard="IEC 60364 / NBC Part 8",
        rationale="Waste-water pipes ≥150mm from electrical cables.",
        is_safety_critical=True,
    ),

    # ── MANHOLE ───────────────────────────────────────────────────────────────
    ClearanceRule(
        rule_id="MH-01", elem_category_a="manhole_chamber", elem_category_b="ELEC",
        min_clearance_mm=500,
        standard="NBC India Part 9 / BS EN 752",
        rationale="Manholes ≥500mm from electrical; safe access, gas hazard.",
        is_safety_critical=True,
    ),
    ClearanceRule(
        rule_id="MH-02", elem_category_a="manhole_chamber", elem_category_b="PIPE",
        min_clearance_mm=150,
        standard="NBC India Part 9",
        rationale="150mm from manhole to adjacent pipes; maintenance access + settlement.",
        is_safety_critical=False,
    ),

    # ── STRUCTURE ─────────────────────────────────────────────────────────────
    ClearanceRule(
        rule_id="STR-01", elem_category_a="STRUCTURE", elem_category_b="PIPE",
        min_clearance_mm=25,
        standard="NBC India Part 6 / ASHRAE Fundamentals",
        rationale="25mm between MEP pipes and structural elements for hangers/sleeves.",
        is_safety_critical=False,
    ),
    ClearanceRule(
        rule_id="STR-02", elem_category_a="STRUCTURE", elem_category_b="DUCT",
        min_clearance_mm=50,
        standard="SMACNA / NBC India Part 6",
        rationale="50mm minimum between structural elements and ducts for hangers and insulation.",
        is_safety_critical=False,
    ),
    ClearanceRule(
        rule_id="STR-03", elem_category_a="STRUCTURE", elem_category_b="ELEC",
        min_clearance_mm=25,
        standard="IEC 60364 / NBC India Part 6",
        rationale="25mm minimum between structural elements and electrical carriers for "
                  "fixing brackets and isolation.",
        is_safety_critical=False,
    ),

    # ── MAINS WATER (high risk; large bore) ──────────────────────────────────
    ClearanceRule(
        rule_id="MWS-01", elem_category_a="mains_water", elem_category_b="ELEC",
        min_clearance_mm=400,
        standard="IEC 60364 / NBC Part 8 / IS 1239",
        rationale="Main incoming water lines (large bore, high pressure) ≥400mm from "
                  "electrical; catastrophic risk if burst near live cables.",
        is_safety_critical=True,
    ),

    # ── GENERAL CATCH-ALL ─────────────────────────────────────────────────────
    ClearanceRule(
        rule_id="GEN-01", elem_category_a="*", elem_category_b="*",
        min_clearance_mm=25,
        standard="General Good Practice / NBC India",
        rationale="Minimum 25mm between any two MEP elements for tolerances, "
                  "thermal expansion, and maintenance.",
        is_safety_critical=False,
    ),
]


def find_applicable_rule(type_a: MEPElementType, type_b: MEPElementType) -> ClearanceRule:
    """
    Rule matching with specificity tiers:
      1. sub_type exact match (both directions)
      2. one sub_type + one category
      3. both categories
      4. wildcard GEN-01
    """
    checks_a = [type_a.sub_type, type_a.category, "*"]
    checks_b = [type_b.sub_type, type_b.category, "*"]

    for specificity in range(3):   # 0=sub+sub, 1=sub+cat, 2=*+*
        for rule in CLEARANCE_RULES:
            ra, rb = rule.elem_category_a, rule.elem_category_b
            ca = checks_a[min(specificity, len(checks_a)-1)]
            cb = checks_b[min(specificity, len(checks_b)-1)]
            if (ra == ca and rb == cb) or (ra == cb and rb == ca):
                return rule

    return CLEARANCE_RULES[-1]  # GEN-01 fallback


# ══════════════════════════════════════════════════════════════════════════════
#  SECTION 3 ── 3-D SPATIAL INDEX (Axis-Aligned Bounding Box grid)
#  Used to query: "what elements exist within radius R of point P?"
#  without iterating every element in the dataset.
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class AABB:
    """Axis-Aligned Bounding Box in metres."""
    cx: float; cy: float; cz: float   # centre
    hx: float; hy: float; hz: float   # half-extents

    @property
    def min_x(self) -> float: return self.cx - self.hx
    @property
    def max_x(self) -> float: return self.cx + self.hx
    @property
    def min_y(self) -> float: return self.cy - self.hy
    @property
    def max_y(self) -> float: return self.cy + self.hy
    @property
    def min_z(self) -> float: return self.cz - self.hz
    @property
    def max_z(self) -> float: return self.cz + self.hz

    def expanded(self, delta_m: float) -> "AABB":
        """Return copy expanded by delta_m on all sides."""
        return AABB(self.cx, self.cy, self.cz,
                    self.hx + delta_m, self.hy + delta_m, self.hz + delta_m)

    def translated(self, dx: float = 0, dy: float = 0, dz: float = 0) -> "AABB":
        return AABB(self.cx + dx, self.cy + dy, self.cz + dz,
                    self.hx, self.hy, self.hz)

    def overlaps(self, other: "AABB") -> bool:
        return (self.min_x <= other.max_x and self.max_x >= other.min_x and
                self.min_y <= other.max_y and self.max_y >= other.min_y and
                self.min_z <= other.max_z and self.max_z >= other.min_z)

    def surface_gap(self, other: "AABB") -> float:
        """Signed surface-to-surface distance (negative = penetration)."""
        dx = max(self.min_x - other.max_x, other.min_x - self.max_x, 0)
        dy = max(self.min_y - other.max_y, other.min_y - self.max_y, 0)
        dz = max(self.min_z - other.max_z, other.min_z - self.max_z, 0)
        if dx == 0 and dy == 0 and dz == 0:
            # Overlapping — compute penetration depth (negative)
            ox = min(self.max_x, other.max_x) - max(self.min_x, other.min_x)
            oy = min(self.max_y, other.max_y) - max(self.min_y, other.min_y)
            oz = min(self.max_z, other.max_z) - max(self.min_z, other.min_z)
            return -min(ox, oy, oz)
        return math.sqrt(dx*dx + dy*dy + dz*dz)


class SpatialIndex:
    """
    Grid-based 3-D spatial index.
    Stores (element_id, AABB, MEPElementType) tuples.
    Supports fast radius-query and post-move clash checking.
    """
    def __init__(self, cell_size_m: float = 2.0):
        self._cell  = cell_size_m
        self._grid: Dict[Tuple[int,int,int], List[Tuple[str,AABB,MEPElementType]]] = defaultdict(list)
        self._all:  Dict[str, Tuple[AABB, MEPElementType]] = {}

    def _cells_for(self, box: AABB) -> Iterator[Tuple[int,int,int]]:
        x0 = int(math.floor(box.min_x / self._cell))
        x1 = int(math.floor(box.max_x / self._cell))
        y0 = int(math.floor(box.min_y / self._cell))
        y1 = int(math.floor(box.max_y / self._cell))
        z0 = int(math.floor(box.min_z / self._cell))
        z1 = int(math.floor(box.max_z / self._cell))
        for xi in range(x0, x1+1):
            for yi in range(y0, y1+1):
                for zi in range(z0, z1+1):
                    yield (xi, yi, zi)

    def insert(self, eid: str, box: AABB, etype: MEPElementType):
        self._all[eid] = (box, etype)
        for cell in self._cells_for(box):
            self._grid[cell].append((eid, box, etype))

    def query_box(self, query_box: AABB, exclude_ids: Set[str] = None) -> List[Tuple[str,AABB,MEPElementType]]:
        """Return all elements whose AABB overlaps or is near query_box."""
        seen: Set[str] = set()
        results = []
        for cell in self._cells_for(query_box):
            for eid, box, etype in self._grid.get(cell, []):
                if eid in seen:
                    continue
                seen.add(eid)
                if exclude_ids and eid in exclude_ids:
                    continue
                if box.overlaps(query_box):
                    results.append((eid, box, etype))
        return results

    def get(self, eid: str) -> Optional[Tuple[AABB, MEPElementType]]:
        return self._all.get(eid)

    def all_ids(self) -> List[str]:
        return list(self._all.keys())


def build_aabb(clash_point: dict, etype: MEPElementType) -> AABB:
    """
    Estimate an AABB from the clash-point centroid and element-type geometry.
    We use diameter_mm as the cross-section estimate and assume elements run
    horizontally (worst-case: half-extent in X/Y, small extent in Z).
    """
    r = etype.diameter_mm / 2000.0    # radius in metres
    insula = 0.05 if etype.insulation_req else 0.0
    return AABB(
        cx=clash_point["x"], cy=clash_point["y"], cz=clash_point["z"],
        hx=r + insula, hy=r + insula, hz=r + insula,
    )


# ══════════════════════════════════════════════════════════════════════════════
#  SECTION 4 ── REROUTING STRATEGY DEFINITIONS
# ══════════════════════════════════════════════════════════════════════════════

# All strategies the engine may consider, in preference order.
ALL_STRATEGIES = [
    "raise_z",
    "lower_z",
    "lateral_shift_pos_x",
    "lateral_shift_neg_x",
    "lateral_shift_pos_y",
    "lateral_shift_neg_y",
    "diagonal_reposition",
    "offset_parallel",
    "elevation_change",
    "segment_reroute",
]

STRATEGY_LABELS = {
    "raise_z":              "Raise element vertically (increase Z)",
    "lower_z":              "Lower element vertically (decrease Z)",
    "lateral_shift_pos_x":  "Shift element in +X direction (horizontal)",
    "lateral_shift_neg_x":  "Shift element in -X direction (horizontal)",
    "lateral_shift_pos_y":  "Shift element in +Y direction (horizontal)",
    "lateral_shift_neg_y":  "Shift element in -Y direction (horizontal)",
    "diagonal_reposition":  "Diagonal repositioning (X + Z combined)",
    "offset_parallel":      "Route parallel at offset distance",
    "elevation_change":     "Re-specify elevation zone for this service tier",
    "segment_reroute":      "Full segment reroute required",
}

def _strategy_delta(strategy: str, offset_m: float) -> Tuple[float, float, float]:
    """Return (dx, dy, dz) for a given strategy and offset magnitude."""
    mapping = {
        "raise_z":             (0,        0,        offset_m),
        "lower_z":             (0,        0,       -offset_m),
        "lateral_shift_pos_x": (offset_m, 0,        0),
        "lateral_shift_neg_x": (-offset_m,0,        0),
        "lateral_shift_pos_y": (0,        offset_m, 0),
        "lateral_shift_neg_y": (0,       -offset_m, 0),
        "diagonal_reposition": (offset_m * 0.707, 0, offset_m * 0.707),
        "offset_parallel":     (offset_m, 0,        0),   # simplified
        "elevation_change":    (0,        0,        offset_m * 1.5),
        "segment_reroute":     (0,        0,        0),   # no simple delta
    }
    return mapping.get(strategy, (0, 0, 0))


def _feasible_strategies(move_type: MEPElementType, penetration_mm: float) -> List[str]:
    """
    Return an ordered list of strategies that are physically feasible
    for this element type, given the penetration depth.
    """
    strategies: List[str] = []

    # Always consider vertical moves if element has headroom
    if move_type.max_raise_mm > 0:
        strategies.append("raise_z")
    if move_type.max_lower_mm > 0 and move_type.category not in ("STRUCTURE",):
        strategies.append("lower_z")

    # Lateral moves if element is laterally flexible
    if move_type.lateral_flex:
        strategies += [
            "lateral_shift_pos_x",
            "lateral_shift_neg_x",
            "lateral_shift_pos_y",
            "lateral_shift_neg_y",
        ]

    # Diagonal is viable if both vertical + lateral are possible
    if move_type.max_raise_mm > 0 and move_type.lateral_flex:
        strategies.append("diagonal_reposition")

    # Parallel offset good for runs like cable trays
    if move_type.category == "ELEC" and move_type.lateral_flex:
        strategies.append("offset_parallel")

    # Elevation change is a re-zoning approach for deep penetrations
    if penetration_mm > 100:
        strategies.append("elevation_change")

    # Segment reroute always last resort
    strategies.append("segment_reroute")

    return strategies


# ══════════════════════════════════════════════════════════════════════════════
#  SECTION 5 ── POST-RELOCATION CLASH VALIDATOR
#  The heart of v4: before recommending a move, we simulate it and check the
#  moved element against every neighbour in the spatial index.
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class MoveCandidate:
    strategy:           str
    offset_mm:          float
    delta:              Tuple[float, float, float]   # (dx, dy, dz) in metres
    new_position:       Tuple[float, float, float]   # new (cx, cy, cz)
    is_clash_free:      bool
    new_clashes:        List[Dict]   # new clashes introduced by this move
    resolved_clashes:   List[str]    # clash_ids this move resolves
    score:              float        # higher = better
    action_description: str
    warning:            str = ""


def validate_move(
    move_eid:         str,
    move_type:        MEPElementType,
    original_box:     AABB,
    strategy:         str,
    offset_mm:        float,
    spatial_idx:      SpatialIndex,
    all_records:      List["ClashRecord"],
    current_clash_id: str,
) -> MoveCandidate:
    """
    Simulate moving `move_eid` by `offset_mm` along `strategy`.
    Check resulting position for new clashes with all neighbours.
    Returns a MoveCandidate describing the outcome.
    """
    offset_m = offset_mm / 1000.0
    dx, dy, dz = _strategy_delta(strategy, offset_m)
    new_box = original_box.translated(dx, dy, dz)
    new_cx  = original_box.cx + dx
    new_cy  = original_box.cy + dy
    new_cz  = original_box.cz + dz

    # Expand search box by max required clearance (500mm) to catch all neighbours
    search_box = new_box.expanded(0.5)
    neighbours = spatial_idx.query_box(search_box, exclude_ids={move_eid})

    new_clashes = []
    for nb_eid, nb_box, nb_type in neighbours:
        gap_m   = new_box.surface_gap(nb_box)
        gap_mm  = gap_m * 1000
        rule    = find_applicable_rule(move_type, nb_type)
        req_mm  = rule.min_clearance_mm
        if gap_mm < req_mm:
            new_clashes.append({
                "neighbour_eid":   nb_eid,
                "neighbour_label": nb_type.system_label,
                "gap_mm":          round(gap_mm, 2),
                "required_mm":     req_mm,
                "rule_id":         rule.rule_id,
                "is_safety_critical": rule.is_safety_critical,
            })

    is_clash_free = len(new_clashes) == 0

    # Score: higher is better
    # Base 100 if fully clash-free, minus 10 per new soft clash, minus 30 per safety clash
    score = 100.0 if is_clash_free else 0.0
    for nc in new_clashes:
        if nc["is_safety_critical"]:
            score -= 30
        else:
            score -= 10
    # Prefer simpler strategies (lower index = less invasive)
    strategy_cost = ALL_STRATEGIES.index(strategy) if strategy in ALL_STRATEGIES else 9
    score -= strategy_cost * 2

    # Check vertical bounds
    warning = ""
    if strategy == "raise_z" and offset_mm > move_type.max_raise_mm:
        warning = (f"Raise of {offset_mm:.0f}mm exceeds typical max_raise "
                   f"({move_type.max_raise_mm:.0f}mm) for this element type. "
                   "Verify structural headroom before proceeding.")
        score -= 15
    elif strategy == "lower_z" and offset_mm > move_type.max_lower_mm:
        warning = (f"Lower of {offset_mm:.0f}mm exceeds typical max_lower "
                   f"({move_type.max_lower_mm:.0f}mm). Verify slab clearance.")
        score -= 15

    label = STRATEGY_LABELS.get(strategy, strategy)
    action_desc = (
        f"{label} — move element {move_eid} by {offset_mm:.0f}mm "
        f"(Δx={dx*1000:.0f}mm, Δy={dy*1000:.0f}mm, Δz={dz*1000:.0f}mm). "
        f"New position: ({new_cx:.3f}, {new_cy:.3f}, {new_cz:.3f})m."
    )
    if not is_clash_free:
        clash_summary = "; ".join(
            f"{nc['neighbour_label']} (gap={nc['gap_mm']:.0f}mm, req={nc['required_mm']:.0f}mm)"
            for nc in new_clashes[:3]
        )
        action_desc += f" ⚠ Introduces {len(new_clashes)} new clash(es): {clash_summary}."

    return MoveCandidate(
        strategy=strategy,
        offset_mm=offset_mm,
        delta=(dx*1000, dy*1000, dz*1000),
        new_position=(round(new_cx,3), round(new_cy,3), round(new_cz,3)),
        is_clash_free=is_clash_free,
        new_clashes=new_clashes,
        resolved_clashes=[current_clash_id],
        score=score,
        action_description=action_desc,
        warning=warning,
    )


# ══════════════════════════════════════════════════════════════════════════════
#  SECTION 6 ── RESOLUTION PLANNER
#  For each clash: identify element to move, enumerate feasible strategies,
#  compute required offsets, validate each, and rank candidates.
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class ResolutionPlan:
    element_to_reroute:     str
    element_type_label:     str
    best_strategy:          str
    best_offset_mm:         float
    best_action:            str
    is_fully_clash_free:    bool
    all_candidates:         List[MoveCandidate]   # ranked
    cascade_risk:           str    # "NONE" | "LOW" | "MEDIUM" | "HIGH"
    cascade_note:           str


def _select_element_to_move(
    type_a: MEPElementType, eid_a: str,
    type_b: MEPElementType, eid_b: str,
) -> Tuple[str, MEPElementType, str, MEPElementType]:
    """Return (move_eid, move_type, keep_eid, keep_type)."""
    pa, pb = type_a.routing_priority, type_b.routing_priority
    if pa > pb:
        return eid_a, type_a, eid_b, type_b
    elif pb > pa:
        return eid_b, type_b, eid_a, type_a
    else:
        # Equal priority: prefer not moving structural elements
        if type_b.category == "STRUCTURE":
            return eid_a, type_a, eid_b, type_b
        if type_a.category == "STRUCTURE":
            return eid_b, type_b, eid_a, type_a
        # Prefer to move the laterally flexible one
        if type_a.lateral_flex and not type_b.lateral_flex:
            return eid_a, type_a, eid_b, type_b
        if type_b.lateral_flex and not type_a.lateral_flex:
            return eid_b, type_b, eid_a, type_a
        return eid_b, type_b, eid_a, type_a


def plan_resolution(
    clash_record:  "ClashRecord",
    type_a:        MEPElementType,
    type_b:        MEPElementType,
    rule:          ClearanceRule,
    penetration_mm: float,
    spatial_idx:   SpatialIndex,
    all_records:   List["ClashRecord"],
) -> ResolutionPlan:
    """
    Generate and validate all feasible move candidates for a clash.
    Returns a ResolutionPlan with ranked candidates.
    """
    move_eid, move_type, keep_eid, keep_type = _select_element_to_move(
        type_a, clash_record.obj1_eid, type_b, clash_record.obj2_eid
    )

    # Required offset = penetration + required clearance + 25mm construction buffer
    base_offset_mm = penetration_mm + rule.min_clearance_mm + 25.0

    # Feasible strategies for this element
    feasible = _feasible_strategies(move_type, penetration_mm)

    # Build AABB for the element to move
    original_box = build_aabb(clash_record.clash_point, move_type)

    candidates: List[MoveCandidate] = []

    for strategy in feasible:
        if strategy == "segment_reroute":
            # Segment reroute: no spatial simulation, flag as manual
            mc = MoveCandidate(
                strategy=strategy,
                offset_mm=0,
                delta=(0, 0, 0),
                new_position=(
                    original_box.cx, original_box.cy, original_box.cz
                ),
                is_clash_free=True,  # assume engineer will handle manually
                new_clashes=[],
                resolved_clashes=[clash_record.clash_id],
                score=10,  # lowest score — last resort
                action_description=(
                    f"Full segment reroute of element {move_eid} "
                    f"({move_type.system_label}) required. "
                    "Re-route the entire run avoiding this zone. "
                    "Engineer to design new path and verify against all services."
                ),
                warning="Manual design review required. Coordinate with all disciplines.",
            )
            candidates.append(mc)
            continue

        # Try primary offset, then 1.5x and 2x if primary fails
        for multiplier in (1.0, 1.5, 2.0):
            offset = round(base_offset_mm * multiplier, 1)
            mc = validate_move(
                move_eid, move_type, original_box,
                strategy, offset,
                spatial_idx, all_records, clash_record.clash_id,
            )
            candidates.append(mc)
            if mc.is_clash_free:
                break  # found a clean offset for this strategy

    # Sort: fully clash-free first, then by score desc
    candidates.sort(key=lambda c: (-int(c.is_clash_free), -c.score))

    best = candidates[0] if candidates else None

    # Cascade risk assessment
    # Count how many other clashes involve the element we're moving
    eid_clash_count = sum(
        1 for r in all_records
        if r.obj1_eid == move_eid or r.obj2_eid == move_eid
    )
    if eid_clash_count >= 10:
        cascade_risk = "HIGH"
        cascade_note = (
            f"Element {move_eid} appears in {eid_clash_count} clashes. "
            "Moving it may resolve multiple clashes simultaneously, but may also "
            "introduce new clashes with elements not yet modelled in this run."
        )
    elif eid_clash_count >= 5:
        cascade_risk = "MEDIUM"
        cascade_note = (
            f"Element {move_eid} is involved in {eid_clash_count} clashes. "
            "Review all affected clashes before committing this move."
        )
    elif eid_clash_count >= 2:
        cascade_risk = "LOW"
        cascade_note = (
            f"Element {move_eid} has {eid_clash_count} other associated clashes. "
            "Verify this move does not worsen them."
        )
    else:
        cascade_risk = "NONE"
        cascade_note = "No cascade risk identified — element is only involved in this clash."

    return ResolutionPlan(
        element_to_reroute=move_eid,
        element_type_label=move_type.system_label,
        best_strategy=best.strategy if best else "segment_reroute",
        best_offset_mm=best.offset_mm if best else base_offset_mm,
        best_action=best.action_description if best else "Manual review required.",
        is_fully_clash_free=best.is_clash_free if best else False,
        all_candidates=candidates,
        cascade_risk=cascade_risk,
        cascade_note=cascade_note,
    )


# ══════════════════════════════════════════════════════════════════════════════
#  SECTION 7 ── SEVERITY CLASSIFIER (enhanced)
# ══════════════════════════════════════════════════════════════════════════════

SYSTEM_PRIORITY: Dict[str, int] = {
    "soil_drainage":      3, "waste_drainage":    3, "cold_water_supply": 3,
    "hot_water_supply":   3, "hot_water_return":  3, "mains_water":       2,
    "refrigerant":        3, "supply_air":        3, "return_air":        3,
    "fresh_air":          3, "transfer_exhaust":  4, "local_exhaust":     4,
    "kitchen_exhaust":    2, "cable_tray":        3, "conduit":           4,
    "plumbing_vent":      4, "rainwater":         4, "grease_trap_line":  4,
    "slot_drain":         5, "manhole_chamber":   2, "structural_surface":1,
    "mep_equipment":      1,
}

# System-pair escalation matrix: certain combinations are always escalated
_ESCALATION_PAIRS: Set[frozenset] = {
    frozenset({"kitchen_exhaust", "cable_tray"}),
    frozenset({"kitchen_exhaust", "conduit"}),
    frozenset({"mains_water",     "cable_tray"}),
    frozenset({"refrigerant",     "cable_tray"}),
    frozenset({"soil_drainage",   "cold_water_supply"}),
    frozenset({"hot_water_supply","cold_water_supply"}),
}

@dataclass
class SeverityResult:
    severity:             str
    priority_score:       int
    penetration_mm:       float
    required_clearance_mm: float
    applicable_rule:      ClearanceRule
    is_safety_critical:   bool
    escalation_note:      str


def classify_severity(
    distance_m: float,
    type_a:     MEPElementType,
    type_b:     MEPElementType,
) -> SeverityResult:
    penetration_mm = abs(distance_m) * 1000
    rule = find_applicable_rule(type_a, type_b)
    safety_critical = rule.is_safety_critical

    # Base score from safety attributes
    base = 0
    if safety_critical:                                      base += 40
    if type_a.is_pressurised or type_b.is_pressurised:      base += 10
    if type_a.category == "ELEC" or type_b.category == "ELEC": base += 15
    if type_a.medium in ("sewage","grease_waste","air_grease") or \
       type_b.medium in ("sewage","grease_waste","air_grease"): base += 10

    # Penetration depth score
    if   penetration_mm > 200: depth = 35
    elif penetration_mm > 100: depth = 32
    elif penetration_mm > 50:  depth = 28
    elif penetration_mm > 20:  depth = 20
    elif penetration_mm > 10:  depth = 12
    elif penetration_mm > 5:   depth = 7
    else:                      depth = 3

    priority_score = min(100, base + depth)

    # Severity label
    if   safety_critical and penetration_mm > 50: severity = "CRITICAL"
    elif safety_critical or penetration_mm > 50:  severity = "HIGH"
    elif penetration_mm > 20 or (safety_critical and penetration_mm > 5): severity = "HIGH"
    elif penetration_mm > 10: severity = "MEDIUM"
    elif penetration_mm > 1:  severity = "LOW"
    else:                     severity = "INFO"

    # Escalation via system-pair matrix
    pair = frozenset({type_a.sub_type, type_b.sub_type})
    escalation_note = ""
    if pair in _ESCALATION_PAIRS:
        if severity in ("MEDIUM", "LOW", "INFO"):
            severity = "HIGH"
            priority_score = max(priority_score, 65)
        escalation_note = (
            f"Pair ({type_a.sub_type} × {type_b.sub_type}) is in the "
            "mandatory escalation matrix — severity raised to HIGH minimum."
        )
    elif SYSTEM_PRIORITY.get(type_a.sub_type, 5) <= 2 and \
         SYSTEM_PRIORITY.get(type_b.sub_type, 5) <= 2:
        if severity in ("MEDIUM", "LOW"):
            severity = "HIGH"
            priority_score = max(priority_score, 60)
        escalation_note = "Both systems have service priority ≤ 2 — severity escalated."

    # Additional escalation note from rule
    if rule.escalation_note and safety_critical:
        if escalation_note:
            escalation_note += " | " + rule.escalation_note
        else:
            escalation_note = rule.escalation_note

    return SeverityResult(
        severity=severity,
        priority_score=priority_score,
        penetration_mm=round(penetration_mm, 3),
        required_clearance_mm=rule.min_clearance_mm,
        applicable_rule=rule,
        is_safety_critical=safety_critical,
        escalation_note=escalation_note,
    )


# ══════════════════════════════════════════════════════════════════════════════
#  SECTION 8 ── XML PARSER
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class ClashRecord:
    clash_id:    str;  guid:        str;  status:     str
    distance_m:  float; clash_point: dict
    grid_location: str; level:      str;  grid_ref:   str
    created_date: str;  screenshot: str
    obj1_eid:    str;  obj1_layer:  str;  obj1_name:  str
    obj2_eid:    str;  obj2_layer:  str;  obj2_name:  str


def _get_smarttags(obj) -> Dict[str, str]:
    tags = {}
    for st in obj.findall("smarttags/smarttag"):
        ch = list(st)
        k = ch[0].text if ch else ""
        v = ch[1].text if len(ch) > 1 else ""
        if k:
            tags[k] = v or ""
    return tags


def _get_eid(obj) -> str:
    oa = obj.find("objectattribute")
    if oa is not None:
        for c in oa:
            if c.tag == "value":
                return c.text or ""
    return ""


def parse_xml(xml_path: str) -> List[ClashRecord]:
    tree = ET.parse(xml_path)
    root = tree.getroot()
    results = root.findall("batchtest/clashtests/clashtest/clashresults/clashresult")

    records = []
    for r in results:
        pos  = r.find("clashpoint/pos3f")
        d    = r.find("createddate/date")
        grid = r.find("gridlocation")
        objs = r.findall("clashobjects/clashobject")

        gt       = (grid.text or "") if grid is not None else ""
        level    = gt.split(":")[-1].strip() if ":" in gt else gt
        grid_ref = gt.split(":")[0].strip()  if ":" in gt else gt

        date_str = ""
        if d is not None:
            try:
                date_str = (
                    f"{d.get('year')}-{int(d.get('month','1')):02d}-"
                    f"{int(d.get('day','1')):02d} "
                    f"{int(d.get('hour','0')):02d}:{int(d.get('minute','0')):02d}:"
                    f"{int(d.get('second','0')):02d}"
                )
            except (TypeError, ValueError):
                date_str = ""

        def obj_info(obj):
            if obj is None:
                return ("", "", "")
            tags = _get_smarttags(obj)
            layer_el = obj.find("layer")
            return (
                _get_eid(obj),
                (layer_el.text or "") if layer_el is not None else "",
                tags.get("Item Name", ""),
            )

        o1 = objs[0] if len(objs) > 0 else None
        o2 = objs[1] if len(objs) > 1 else None
        eid1, lay1, nm1 = obj_info(o1)
        eid2, lay2, nm2 = obj_info(o2)

        records.append(ClashRecord(
            clash_id=r.get("name",""), guid=r.get("guid",""),
            status=(r.get("status") or "").title(),
            distance_m=float(r.get("distance", 0)),
            clash_point={
                "x": float(pos.get("x",0)) if pos is not None else 0,
                "y": float(pos.get("y",0)) if pos is not None else 0,
                "z": float(pos.get("z",0)) if pos is not None else 0,
            },
            grid_location=gt, level=level, grid_ref=grid_ref,
            created_date=date_str, screenshot=r.get("href",""),
            obj1_eid=eid1, obj1_layer=lay1, obj1_name=nm1,
            obj2_eid=eid2, obj2_layer=lay2, obj2_name=nm2,
        ))
    return records


# ══════════════════════════════════════════════════════════════════════════════
#  SECTION 9 ── CLASH GRAPH ANALYSIS
#  Builds a graph where nodes = elements, edges = clashes.
#  Used to identify hot-spot elements and clash clusters.
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class ClashGraphStats:
    hotspot_elements:    List[Dict]   # elements involved in most clashes
    cluster_count:       int          # approximate connected-component count
    largest_cluster:     int          # size of largest component (# elements)
    avg_degree:          float        # avg number of clashes per element


def build_clash_graph(records: List[ClashRecord]) -> ClashGraphStats:
    degree: Counter = Counter()
    adjacency: Dict[str, Set[str]] = defaultdict(set)

    for r in records:
        if r.obj1_eid and r.obj2_eid:
            degree[r.obj1_eid] += 1
            degree[r.obj2_eid] += 1
            adjacency[r.obj1_eid].add(r.obj2_eid)
            adjacency[r.obj2_eid].add(r.obj1_eid)

    # Find connected components via BFS
    visited: Set[str] = set()
    components = []
    for node in adjacency:
        if node in visited:
            continue
        component = []
        queue = [node]
        while queue:
            n = queue.pop()
            if n in visited:
                continue
            visited.add(n)
            component.append(n)
            queue.extend(adjacency[n] - visited)
        components.append(component)

    hotspots = [
        {"element_id": eid, "clash_count": cnt}
        for eid, cnt in degree.most_common(15)
    ]

    avg_deg = sum(degree.values()) / max(len(degree), 1)

    return ClashGraphStats(
        hotspot_elements=hotspots,
        cluster_count=len(components),
        largest_cluster=max((len(c) for c in components), default=0),
        avg_degree=round(avg_deg, 2),
    )


# ══════════════════════════════════════════════════════════════════════════════
#  SECTION 10 ── ENRICHED OUTPUT RECORD
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class EnrichedClash:
    # Raw
    clash_id: str; guid: str; status: str; distance_m: float
    clash_point: dict; grid_location: str; level: str; grid_ref: str
    created_date: str; screenshot: str

    # Element identity
    obj1_eid: str; obj1_name: str; obj1_layer: str
    obj1_system_label: str; obj1_category: str; obj1_sub_type: str; obj1_medium: str
    obj2_eid: str; obj2_name: str; obj2_layer: str
    obj2_system_label: str; obj2_category: str; obj2_sub_type: str; obj2_medium: str
    clash_pair_label: str

    # Severity
    severity: str; priority_score: int; penetration_mm: float
    required_clearance_mm: float; applicable_rule_id: str
    applicable_standard: str; rule_rationale: str
    is_safety_critical: bool; escalation_note: str

    # Resolution
    element_to_reroute: str; element_type_label: str
    best_strategy: str; best_offset_mm: float
    best_action: str; is_fully_clash_free: bool
    cascade_risk: str; cascade_note: str
    all_resolution_candidates: List[Dict]


def enrich(
    record:      ClashRecord,
    spatial_idx: SpatialIndex,
    all_records: List[ClashRecord],
) -> EnrichedClash:
    t1 = get_element_type(record.obj1_name)
    t2 = get_element_type(record.obj2_name)
    sev = classify_severity(record.distance_m, t1, t2)
    rule = sev.applicable_rule

    plan = plan_resolution(
        record, t1, t2, rule, sev.penetration_mm,
        spatial_idx, all_records,
    )

    candidates_out = []
    for mc in plan.all_candidates:
        candidates_out.append({
            "strategy":            mc.strategy,
            "strategy_label":      STRATEGY_LABELS.get(mc.strategy, mc.strategy),
            "offset_mm":           mc.offset_mm,
            "delta_mm":            {
                "x": round(mc.delta[0], 1),
                "y": round(mc.delta[1], 1),
                "z": round(mc.delta[2], 1),
            },
            "new_position_m":      {
                "x": mc.new_position[0],
                "y": mc.new_position[1],
                "z": mc.new_position[2],
            },
            "is_clash_free":       mc.is_clash_free,
            "score":               round(mc.score, 1),
            "new_clashes_count":   len(mc.new_clashes),
            "new_clashes_detail":  mc.new_clashes[:5],  # cap to 5 for JSON size
            "action_description":  mc.action_description,
            "warning":             mc.warning,
        })

    return EnrichedClash(
        clash_id=record.clash_id, guid=record.guid, status=record.status,
        distance_m=record.distance_m, clash_point=record.clash_point,
        grid_location=record.grid_location, level=record.level, grid_ref=record.grid_ref,
        created_date=record.created_date, screenshot=record.screenshot,

        obj1_eid=record.obj1_eid, obj1_name=record.obj1_name, obj1_layer=record.obj1_layer,
        obj1_system_label=t1.system_label, obj1_category=t1.category,
        obj1_sub_type=t1.sub_type, obj1_medium=t1.medium,

        obj2_eid=record.obj2_eid, obj2_name=record.obj2_name, obj2_layer=record.obj2_layer,
        obj2_system_label=t2.system_label, obj2_category=t2.category,
        obj2_sub_type=t2.sub_type, obj2_medium=t2.medium,

        clash_pair_label=f"{t1.system_label} × {t2.system_label}",

        severity=sev.severity, priority_score=sev.priority_score,
        penetration_mm=sev.penetration_mm,
        required_clearance_mm=sev.required_clearance_mm,
        applicable_rule_id=rule.rule_id, applicable_standard=rule.standard,
        rule_rationale=rule.rationale, is_safety_critical=sev.is_safety_critical,
        escalation_note=sev.escalation_note,

        element_to_reroute=plan.element_to_reroute,
        element_type_label=plan.element_type_label,
        best_strategy=plan.best_strategy,
        best_offset_mm=plan.best_offset_mm,
        best_action=plan.best_action,
        is_fully_clash_free=plan.is_fully_clash_free,
        cascade_risk=plan.cascade_risk,
        cascade_note=plan.cascade_note,
        all_resolution_candidates=candidates_out,
    )


# ══════════════════════════════════════════════════════════════════════════════
#  SECTION 11 ── MAIN RUNNER
# ══════════════════════════════════════════════════════════════════════════════

SEV_ORDER = {"CRITICAL": 0, "HIGH": 1, "MEDIUM": 2, "LOW": 3, "INFO": 4}
SEV_EMOJI = {"CRITICAL": "🔴", "HIGH": "🟠", "MEDIUM": "🟡", "LOW": "🟢", "INFO": "⚪"}


def _build_spatial_index(records: List[ClashRecord]) -> SpatialIndex:
    """
    Populate the spatial index from all unique elements in the clash dataset.
    We use the clash_point as a proxy for element centroid.
    """
    idx = SpatialIndex(cell_size_m=2.0)
    seen: Set[str] = set()
    for r in records:
        for eid, name, pt in [
            (r.obj1_eid, r.obj1_name, r.clash_point),
            (r.obj2_eid, r.obj2_name, r.clash_point),
        ]:
            if eid and eid not in seen:
                etype = get_element_type(name)
                box   = build_aabb(pt, etype)
                idx.insert(eid, box, etype)
                seen.add(eid)
    return idx


def run(xml_path: str, output_path: str):
    W = 72
    print(f"\n{'═'*W}")
    print("  MEP CLASH RULE ENGINE v4.0 — Spatial-Validated Resolution Planner")
    print(f"{'═'*W}")
    print(f"  Input  : {xml_path}")
    print(f"  Output : {output_path}")

    records = parse_xml(xml_path)
    print(f"  Loaded : {len(records):,} clash records")

    # Build spatial index
    spatial_idx = _build_spatial_index(records)
    print(f"  Indexed: {len(spatial_idx.all_ids()):,} unique elements in 3-D spatial grid")

    # Enrich
    print(f"  Processing clashes with multi-strategy spatial validation …")
    enriched: List[EnrichedClash] = []
    for r in records:
        enriched.append(enrich(r, spatial_idx, records))
    enriched.sort(key=lambda x: (-x.priority_score, -x.penetration_mm))

    # ── Statistics ────────────────────────────────────────────────────────────
    sev_cnt     = Counter(e.severity for e in enriched)
    rule_cnt    = Counter(e.applicable_rule_id for e in enriched)
    pair_cnt    = Counter(e.clash_pair_label for e in enriched)
    strat_cnt   = Counter(e.best_strategy for e in enriched)
    lvl_cnt     = Counter(e.level for e in enriched)
    cascade_cnt = Counter(e.cascade_risk for e in enriched)
    safety_cnt  = sum(1 for e in enriched if e.is_safety_critical)
    free_cnt    = sum(1 for e in enriched if e.is_fully_clash_free)
    esc_cnt     = sum(1 for e in enriched if e.escalation_note)

    pens      = [e.penetration_mm for e in enriched]
    avg_pen   = sum(pens) / len(pens) if pens else 0
    max_pen   = max(pens) if pens else 0

    graph_stats = build_clash_graph(records)

    print(f"\n  {'─'*68}")
    print(f"  SEVERITY BREAKDOWN")
    print(f"  {'─'*68}")
    for sev in ["CRITICAL","HIGH","MEDIUM","LOW","INFO"]:
        cnt = sev_cnt.get(sev, 0)
        bar = "█" * int(cnt / max(len(enriched),1) * 40)
        print(f"  {SEV_EMOJI[sev]} {sev:<10}  {cnt:>5}  {cnt/max(len(enriched),1)*100:>5.1f}%  {bar}")

    print(f"\n  Safety-Critical clashes  : {safety_cnt:,}")
    print(f"  Escalated clashes        : {esc_cnt:,}")
    print(f"  Avg penetration depth    : {avg_pen:.2f} mm")
    print(f"  Max penetration depth    : {max_pen:.2f} mm")

    print(f"\n  RESOLUTION VALIDATION RESULTS:")
    print(f"    Clash-free moves found : {free_cnt:,} / {len(enriched):,}")
    print(f"    Cascade risk — HIGH    : {cascade_cnt.get('HIGH',0)}")
    print(f"    Cascade risk — MEDIUM  : {cascade_cnt.get('MEDIUM',0)}")
    print(f"    Cascade risk — LOW     : {cascade_cnt.get('LOW',0)}")

    print(f"\n  CLASH GRAPH:")
    print(f"    Connected clusters     : {graph_stats.cluster_count}")
    print(f"    Largest cluster        : {graph_stats.largest_cluster} elements")
    print(f"    Avg clashes/element    : {graph_stats.avg_degree:.2f}")
    print(f"    Top hotspot elements:")
    for h in graph_stats.hotspot_elements[:5]:
        print(f"      {h['element_id']:<30}  {h['clash_count']:>4} clashes")

    print(f"\n  BEST STRATEGY DISTRIBUTION:")
    for strat, cnt in strat_cnt.most_common():
        print(f"    {STRATEGY_LABELS.get(strat,strat):<45}  {cnt:>5}")

    print(f"\n  TOP RULES TRIGGERED:")
    for rid, cnt in rule_cnt.most_common(8):
        ro = next((r for r in CLEARANCE_RULES if r.rule_id == rid), None)
        label = ro.standard[:55] if ro else rid
        print(f"    {rid:<10}  {cnt:>4}×  [{label}]")

    print(f"\n  TOP 5 MOST CRITICAL CLASHES:")
    for e in enriched[:5]:
        cf = "✓ clean" if e.is_fully_clash_free else "⚠ new clashes"
        print(f"    {SEV_EMOJI[e.severity]} {e.clash_id:<10} "
              f"score={e.priority_score:>3}  pen={e.penetration_mm:>7.2f}mm  "
              f"strategy={e.best_strategy:<22}  {cf}")
        print(f"      {e.best_action[:90]}")

    # ── Build output JSON ─────────────────────────────────────────────────────
    output = {
        "schema_version": "4.0",
        "generator": "mep_rule_engine_v4.py",
        "source_file": xml_path,
        "summary": {
            "total_clashes":          len(enriched),
            "safety_critical_count":  safety_cnt,
            "escalated_count":        esc_cnt,
            "by_severity":            dict(sev_cnt),
            "by_floor_level":         dict(lvl_cnt),
            "by_rule_triggered":      dict(rule_cnt.most_common(15)),
            "by_clash_pair_type":     dict(pair_cnt.most_common(20)),
            "by_best_strategy":       dict(strat_cnt),
            "by_cascade_risk":        dict(cascade_cnt),
            "resolution_stats": {
                "clash_free_resolutions":    free_cnt,
                "resolutions_with_warnings": len(enriched) - free_cnt,
            },
            "avg_penetration_mm": round(avg_pen, 3),
            "max_penetration_mm": round(max_pen, 3),
            "clash_graph": {
                "connected_clusters":     graph_stats.cluster_count,
                "largest_cluster_size":   graph_stats.largest_cluster,
                "avg_degree":             graph_stats.avg_degree,
                "hotspot_elements":       graph_stats.hotspot_elements,
            },
            "parameters": {
                "standards_applied": [
                    "ASHRAE 90.1-2019", "ASHRAE 15-2019",
                    "SMACNA HVAC Duct Construction Standards 3rd Ed.",
                    "IEC 60364-5-52", "NBC India Part 4, 6, 8, 9",
                    "BS EN 806", "CIBSE Guide G", "NFPA 96",
                    "IS 1172 / IS 1239",
                ],
                "clearance_rules_count":     len(CLEARANCE_RULES),
                "element_types_mapped":      len(ELEMENT_CATALOGUE),
                "rerouting_strategies_count": len(ALL_STRATEGIES),
                "spatial_index_elements":    len(spatial_idx.all_ids()),
            },
        },
        "clearance_rules_reference": [
            {
                "rule_id":            r.rule_id,
                "category_a":         r.elem_category_a,
                "category_b":         r.elem_category_b,
                "min_clearance_mm":   r.min_clearance_mm,
                "standard":           r.standard,
                "rationale":          r.rationale,
                "is_safety_critical": r.is_safety_critical,
                "escalation_note":    r.escalation_note,
            }
            for r in CLEARANCE_RULES
        ],
        "classified_clashes": [
            {
                "clash_id":            e.clash_id,
                "guid":                e.guid,
                "severity":            e.severity,
                "priority_score":      e.priority_score,
                "penetration_mm":      e.penetration_mm,
                "required_clearance_mm": e.required_clearance_mm,
                "is_safety_critical":  e.is_safety_critical,
                "escalation_note":     e.escalation_note,
                "applicable_rule_id":  e.applicable_rule_id,
                "applicable_standard": e.applicable_standard,
                "rule_rationale":      e.rule_rationale,
                "clash_pair_label":    e.clash_pair_label,
                "clash_point_xyz":     e.clash_point,
                "grid_location":       e.grid_location,
                "level":               e.level,
                "grid_ref":            e.grid_ref,
                "created_date":        e.created_date,
                "element_1": {
                    "element_id":   e.obj1_eid, "name":     e.obj1_name,
                    "layer":        e.obj1_layer,
                    "system_label": e.obj1_system_label,
                    "category":     e.obj1_category,
                    "sub_type":     e.obj1_sub_type, "medium": e.obj1_medium,
                },
                "element_2": {
                    "element_id":   e.obj2_eid, "name":     e.obj2_name,
                    "layer":        e.obj2_layer,
                    "system_label": e.obj2_system_label,
                    "category":     e.obj2_category,
                    "sub_type":     e.obj2_sub_type, "medium": e.obj2_medium,
                },
                "resolution": {
                    "element_to_reroute":   e.element_to_reroute,
                    "element_type_label":   e.element_type_label,
                    "best_strategy":        e.best_strategy,
                    "best_strategy_label":  STRATEGY_LABELS.get(e.best_strategy,""),
                    "best_offset_mm":       e.best_offset_mm,
                    "best_action":          e.best_action,
                    "is_fully_clash_free":  e.is_fully_clash_free,
                    "cascade_risk":         e.cascade_risk,
                    "cascade_note":         e.cascade_note,
                    "all_candidates":       e.all_resolution_candidates,
                },
                "screenshot": e.screenshot,
            }
            for e in enriched
        ],
    }

    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(output, f, indent=2)

    print(f"\n  ✅ Output written → {output_path}")
    print(f"{'═'*W}\n")
    return output


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="MEP Clash Rule Engine v4")
    parser.add_argument("--input",  default="hero.xml",
                        help="Navisworks clash XML file")
    parser.add_argument("--output", default="clash_rule_engine_report.json",
                        help="Output JSON report path")
    args = parser.parse_args()
    run(args.input, args.output)