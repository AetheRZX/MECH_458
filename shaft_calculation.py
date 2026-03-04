import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# =========================================================================
# 1. SIMULATION INPUTS
# =========================================================================
print("Initializing Shaft Analysis...")

# --- Geometry ---
L_Shaft = 3.1           # Total Length (m)
Loc_Bearing_L = 0.2     # Left Bearing Location (m)
Loc_Load = 3.1/2          # Load/Table Center Location (m)
Loc_Bearing_R = 2.8     # Right Bearing Location (m)

d_mm = 60               # Shaft Diameter (mm)
d = d_mm / 1000         # Shaft Diameter (m)

# --- Loads ---
Mass_Table = 50.0       # kg (Frame weight)
Mass_Payload = 115.0    # kg (250 lbs)
Force_Gravity = (Mass_Table + Mass_Payload) * 9.81  # Downward Force (N)

Torque_Rated = 1000.0   # Nm (Normal Operation)
Torque_Stall = 2066.0   # Nm (Crash / Safety Limit)

# --- Keyway Dimensions (DIN 6885 for 60mm Shaft) ---
Key_W = 18 / 1000       # 18mm Width
Key_H = 11 / 1000       # 11mm Height
Key_L = 70 / 1000       # 70mm Length (Assumed)

# --- Stress Concentration Factors ---
Kt_Bend = 2.14          # Keyway Bending Factor
Kt_Torsion = 3.00       # Keyway Torsion Factor

# =========================================================================
# 2. MATERIAL LIBRARY (Properties & Relative Cost)
# =========================================================================
# Format: "Name": [Yield_Strength_Pa, Elastic_Modulus_Pa, Cost_Index]
# Cost Index: 1.0 = Cheapest (1018 Steel). 4.0 = 4x more expensive.
materials = {
    "1018 Mild Steel":      [370e6, 205e9, 1.0],  # Cheap, soft
    "1045 Carbon Steel":    [450e6, 205e9, 1.2],  # Standard shafting (TGP)
    "4140 Steel (Pre-Hard)":[655e6, 210e9, 2.0],  # High strength, common
    "304 Stainless Steel":  [215e6, 193e9, 3.5],  # Corrosion resistant, WEAK
    "17-4 PH Stainless":    [1000e6,196e9, 8.0]   # Aerospace grade, expensive
}

# =========================================================================
# 3. STATICS SOLVER (Simply Supported Beam)
# =========================================================================
# Reaction Forces
Span = Loc_Bearing_R - Loc_Bearing_L
Dist_Load = Loc_Load - Loc_Bearing_L
R_Right = (Force_Gravity * Dist_Load) / Span
R_Left = Force_Gravity - R_Right

# Generate Diagrams
x = np.linspace(0, L_Shaft, 1000)
M = np.zeros_like(x)
T = np.zeros_like(x)

for i, xi in enumerate(x):
    # Moment Calculation
    if xi > Loc_Bearing_L: M[i] += R_Left * (xi - Loc_Bearing_L)
    if xi > Loc_Load:      M[i] -= Force_Gravity * (xi - Loc_Load)
    if xi > Loc_Bearing_R: M[i] += R_Right * (xi - Loc_Bearing_R)
    
    # Torque Calculation (Stall Case)
    if 0 <= xi <= Loc_Load:
        T[i] = Torque_Stall

Max_Moment = np.max(M)
Max_Torque = np.max(T)

# =========================================================================
# 4. MATERIAL COMPARISON LOOP
# =========================================================================
results = []
print(f"\n{'MATERIAL':<25} | {'YIELD (MPa)':<12} | {'SF (STALL)':<10} | {'STATUS':<10} | {'COST'}")
print("-" * 80)

# Geometric Properties
c = d / 2
I = (np.pi * d**4) / 64
J = (np.pi * d**4) / 32

comparison_names = []
comparison_sf = []
comparison_cost = []

for name, props in materials.items():
    Yield = props[0]
    E_Mod = props[1]
    Cost = props[2]
    
    # Stress Calculation (Von Mises at Keyway)
    Sigma = (Max_Moment * c / I) * Kt_Bend
    Tau = (Max_Torque * c / J) * Kt_Torsion
    VonMises = np.sqrt(Sigma**2 + 3*Tau**2)
    
    SF = Yield / VonMises
    
    status = "PASS" if SF >= 1.0 else "FAIL"
    
    # Store for plotting
    comparison_names.append(name)
    comparison_sf.append(SF)
    comparison_cost.append(Cost)
    
    print(f"{name:<25} | {Yield/1e6:<12.0f} | {SF:<10.2f} | {status:<10} | {Cost}x")

# =========================================================================
# 5. VISUALIZATION
# =========================================================================
fig = plt.figure(figsize=(14, 10))
gs = fig.add_gridspec(2, 2)

# --- PLOT 1: FBD (Big & Clear) ---
ax_fbd = fig.add_subplot(gs[0, :])
ax_fbd.set_title(f"Free Body Diagram (Load: {Force_Gravity:.0f} N)", fontsize=16)
ax_fbd.set_xlim(-0.1, L_Shaft + 0.1)
ax_fbd.set_ylim(-0.5, 0.8)
ax_fbd.set_aspect('equal')
ax_fbd.axis('off')

# Draw Shaft
rect = patches.Rectangle((0, -0.1), L_Shaft, 0.2, linewidth=2, edgecolor='k', facecolor='#ddd')
ax_fbd.add_patch(rect)
ax_fbd.text(L_Shaft/2, -0.1, f"Shaft Ø{d_mm}mm", ha='center', va='top', fontsize=12)

# Draw Supports
def draw_support(ax, x, label, force):
    poly = patches.Polygon([[x-0.05, -0.25], [x+0.05, -0.25], [x, -0.1]], closed=True, color='blue')
    ax.add_patch(poly)
    ax.annotate(f"{label}\n{force:.0f} N", xy=(x, -0.25), xytext=(x, -0.5), 
                ha='center', fontsize=12, arrowprops=dict(arrowstyle="->", color='blue', lw=2))

draw_support(ax_fbd, Loc_Bearing_L, "Bearing L", R_Left)
draw_support(ax_fbd, Loc_Bearing_R, "Bearing R", R_Right)

# Draw Load
ax_fbd.annotate(f"LOAD\n{Force_Gravity:.0f} N", xy=(Loc_Load, 0.1), xytext=(Loc_Load, 0.6),
                ha='center', fontsize=14, color='red', weight='bold',
                arrowprops=dict(facecolor='red', shrink=0.05, width=10))
ax_fbd.annotate(f"Stall Torque\n{Torque_Stall:.0f} Nm", xy=(0, 0), xytext=(-0.1, 0.4), color='green',
                arrowprops=dict(arrowstyle="simple,tail_width=0.5,head_width=1.5,head_length=1.5", 
                                connectionstyle="arc3,rad=.4", color='green'))

# --- PLOT 2: STRENGTH COMPARISON (Bar Chart) ---
ax_bar = fig.add_subplot(gs[1, 0])
colors = ['red' if sf < 1.0 else 'orange' if sf < 1.5 else 'green' for sf in comparison_sf]
bars = ax_bar.barh(comparison_names, comparison_sf, color=colors)
ax_bar.axvline(1.0, color='red', linestyle='--', linewidth=2, label="Failure Line")
ax_bar.axvline(1.5, color='orange', linestyle='--', linewidth=1, label="Min Safety Margin")
ax_bar.set_xlabel("Safety Factor (Stall Condition)")
ax_bar.set_title("Material Safety Factor Comparison")
ax_bar.legend()

# Add values to bars
for bar in bars:
    width = bar.get_width()
    ax_bar.text(width + 0.1, bar.get_y() + bar.get_height()/2, 
                f'{width:.2f}', va='center', fontweight='bold')

# --- PLOT 3: COST VS PERFORMANCE ---
ax_scat = fig.add_subplot(gs[1, 1])
ax_scat.scatter(comparison_cost, comparison_sf, s=200, c=colors, edgecolors='k', zorder=2)
ax_scat.set_xlabel("Relative Cost (1.0 = Cheapest)")
ax_scat.set_ylabel("Safety Factor")
ax_scat.set_title("Cost vs. Performance Trade-off")
ax_scat.grid(True, linestyle='--', alpha=0.6)
ax_scat.axhline(1.0, color='red', linestyle='--')

for i, txt in enumerate(comparison_names):
    ax_scat.annotate(txt, (comparison_cost[i], comparison_sf[i]), 
                     xytext=(5, 5), textcoords='offset points')

plt.tight_layout()
plt.show()

# =========================================================================
# 6. KEYWAY CHECK (For the selected/cheapest passing material)
# =========================================================================
print("\n=== KEYWAY ANALYSIS (18x11x70mm) ===")
# Using C1018 Key Stock (Standard) - Yield ~300 MPa
Key_Yield = 300e6
Key_Shear_Yield = Key_Yield * 0.577

Force_Key = Torque_Stall / c
Shear_Stress_Key = Force_Key / (Key_W * Key_L)
Crush_Stress_Key = Force_Key / ((Key_H/2) * Key_L)

SF_Key_Shear = Key_Shear_Yield / Shear_Stress_Key
SF_Key_Crush = Key_Yield / Crush_Stress_Key

print(f"Key Material: C1018 Steel (Sacrificial)")
print(f"Shear Safety Factor:    {SF_Key_Shear:.2f}  [{'PASS' if SF_Key_Shear > 1 else 'FAIL'}]")
print(f"Crushing Safety Factor: {SF_Key_Crush:.2f}  [{'PASS' if SF_Key_Crush > 1 else 'FAIL'}]")
if SF_Key_Shear < 1.0:
    print(">>> WARNING: Key will shear before shaft breaks. This is GOOD (Mechanical Fuse).")