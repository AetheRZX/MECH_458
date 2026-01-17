import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, RadioButtons, TextBox
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def final_physics_sim_inertia():
    # --- System Constants (Base Physics in Metric) ---
    L_x = 2.4  # Length (Along Axis) in meters
    L_y = 1.8  # Width (Lever Arm source) in meters
    Thickness = 0.25 # meters
    
    # Base Weights (Metric)
    m_load_kg_base = 250.0 * 0.453592 
    m_table_kg_base = 500.0 * 0.453592 
    g = 9.81

    # Motion Constants for Inertia Calc
    target_rpm = 1.0
    t_accel = 0.5
    alpha = (target_rpm * 2 * np.pi / 60) / t_accel

    # State
    state = {
        'load_x': 0.0, 
        'load_y': 0.0, 
        'angle': 0.0,
        'table_offset_y': 0.0, 
        'units': 'Metric' # <--- CHANGED TO METRIC DEFAULT
    }
    
    fig = plt.figure(figsize=(16, 9))
    fig.canvas.manager.set_window_title("Airbus MECH 458 - Rotation Simulation")

    # --- Layout Definitions ---
    # Top View (Top Left)
    ax_top   = fig.add_axes([0.05, 0.55, 0.30, 0.40]) 
    # Gauge (Middle Left)
    ax_gauge = fig.add_axes([0.05, 0.20, 0.30, 0.25]) 
    # 3D View (Right Side)
    ax_3d    = fig.add_axes([0.40, 0.15, 0.55, 0.80], projection='3d')
    # Unit Switch (Bottom Left)
    ax_radio = fig.add_axes([0.05, 0.02, 0.10, 0.10]) 

    # --- Slider & Text Box Layout (Bottom Right Area) ---
    # Rotation Row
    ax_sl_ang  = fig.add_axes([0.40, 0.07, 0.40, 0.03])
    ax_box_ang = fig.add_axes([0.85, 0.07, 0.08, 0.03])
    
    # Unbalance Row (Stacked Below)
    ax_sl_off  = fig.add_axes([0.40, 0.03, 0.40, 0.03])
    ax_box_off = fig.add_axes([0.85, 0.03, 0.08, 0.03])

    # --- 1. Top View (Input) ---
    ax_top.set_title("1. Load Placement (Top View)")
    ax_top.set_xlim(-L_x/2 - 0.2, L_x/2 + 0.2)
    ax_top.set_ylim(-L_y/2 - 0.2, L_y/2 + 0.2)
    ax_top.set_aspect('equal')
    ax_top.grid(True, alpha=0.3)
    ax_top.set_xlabel("Meters (Internal)")
    
    table_patch = plt.Rectangle((-L_x/2, -L_y/2), L_x, L_y, facecolor='#e0e0e0', edgecolor='k')
    ax_top.add_patch(table_patch)
    ax_top.axhline(0, color='r', linestyle='--', label="Rotation Axis")
    
    dot_load, = ax_top.plot([0], [0], 'ro', markersize=10, label='Load')
    dot_com, = ax_top.plot([0], [0], 'bx', markersize=10, label='Table CoM')
    
    txt_lever = ax_top.text(0, L_y/2 + 0.1, "", ha='center', fontsize=10, backgroundcolor='white')

    # --- 2. Gauge ---
    ax_gauge.set_title("2. Required Torque Breakdown")
    ax_gauge.set_xlim(0, 1)
    ax_gauge.set_xticks([])
    
    # Bars (Stacked)
    # Order: Inertia (Bottom), Unbalance (Middle), Load Gravity (Top)
    bar_inertia = ax_gauge.bar(0.4, 0, width=0.3, color='green', alpha=0.5, label='Inertia (Accel)')
    bar_table = ax_gauge.bar(0.4, 0, width=0.3, bottom=0, color='blue', alpha=0.6, label='Unbalance')
    bar_load = ax_gauge.bar(0.4, 0, width=0.3, bottom=0, color='orange', label='Load Gravity')
    
    txt_total = ax_gauge.text(0.4, 0, "", ha='center', fontsize=14, fontweight='bold')
    ax_gauge.legend(loc='upper left', fontsize='small')

    # --- 3. 3D Helper ---
    def get_cube_verts(center, dx, dy, dz, angle_deg):
        theta = np.deg2rad(angle_deg)
        c, s = np.cos(theta), np.sin(theta)
        R = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
        
        corners = np.array([
            [-dx/2, -dy/2, -dz/2], [dx/2, -dy/2, -dz/2], [dx/2, dy/2, -dz/2], [-dx/2, dy/2, -dz/2],
            [-dx/2, -dy/2, dz/2],  [dx/2, -dy/2, dz/2],  [dx/2, dy/2, dz/2],  [-dx/2, dy/2, dz/2]
        ])
        rotated_geo = np.dot(corners, R.T)
        final_verts = rotated_geo + center
        
        verts_coll = [
            [final_verts[0], final_verts[1], final_verts[2], final_verts[3]],
            [final_verts[4], final_verts[5], final_verts[6], final_verts[7]],
            [final_verts[0], final_verts[1], final_verts[5], final_verts[4]],
            [final_verts[2], final_verts[3], final_verts[7], final_verts[6]],
            [final_verts[1], final_verts[2], final_verts[6], final_verts[5]],
            [final_verts[4], final_verts[7], final_verts[3], final_verts[0]]
        ]
        return verts_coll

    # --- Update Logic ---
    def update_viz(val=None):
        theta_rad = np.deg2rad(state['angle'])
        c, s = np.cos(theta_rad), np.sin(theta_rad)
        R = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

        # --- PHYSICS CALCS ---
        # 1. Load Gravity
        pos_load_local = np.array([state['load_x'], state['load_y'], Thickness/2])
        pos_load_global = np.dot(R, pos_load_local)
        lever_load_m = abs(pos_load_global[1])
        torque_load_nm = m_load_kg_base * g * lever_load_m
        
        # 2. Table Unbalance Gravity
        pos_table_local = np.array([0, state['table_offset_y'], 0])
        pos_table_global = np.dot(R, pos_table_local)
        lever_table_m = abs(pos_table_global[1])
        torque_table_nm = m_table_kg_base * g * lever_table_m
        
        # 3. Inertia (Dynamic)
        # Table Inertia (Fixed)
        I_table = (1/12) * m_table_kg_base * (L_y**2 + Thickness**2)
        
        # Load Inertia (Variable based on radius)
        # Radius is distance from X-axis = sqrt(y^2 + z^2)
        r_load = np.sqrt(state['load_y']**2 + (Thickness/2)**2)
        I_load = m_load_kg_base * r_load**2
        
        T_inertia_nm = (I_table + I_load) * alpha
        
        # Total
        total_torque_nm = torque_load_nm + torque_table_nm + T_inertia_nm + 25.0 # Friction

        # --- UNIT CONVERSION ---
        is_imp = (state['units'] == 'Imperial')
        
        if is_imp:
            t_factor = 0.73756 # Nm -> lb-ft
            unit_t = "lb-ft"
            l_factor = 39.37 # m -> inches
            unit_l = "in"
            ax_gauge.set_ylim(0, 1500)
        else:
            t_factor = 1.0
            unit_t = "Nm"
            l_factor = 1.0
            unit_l = "m"
            ax_gauge.set_ylim(0, 2000)

        # Update Displays
        disp_total = total_torque_nm * t_factor
        disp_load_t = torque_load_nm * t_factor
        disp_table_t = torque_table_nm * t_factor
        disp_inertia_t = T_inertia_nm * t_factor
        disp_lever = abs(state['load_y']) * l_factor

        # 1. Top View
        dot_load.set_data([state['load_x']], [state['load_y']])
        dot_com.set_data([0], [state['table_offset_y']])
        txt_lever.set_text(f"Load Offset: {disp_lever:.1f} {unit_l}")
        
        # 2. Gauge (Stacked)
        # Stack: Inertia (Bottom) -> Table (Middle) -> Load (Top)
        
        bar_inertia[0].set_height(disp_inertia_t)
        
        bar_table[0].set_height(disp_table_t)
        bar_table[0].set_y(disp_inertia_t)
        
        bar_load[0].set_height(disp_load_t)
        bar_load[0].set_y(disp_inertia_t + disp_table_t)
        
        txt_total.set_text(f"Total: {disp_total:.0f} {unit_t}")
        txt_total.set_position((0.4, disp_total + (100 if is_imp else 150)))

        # 3. 3D View
        ax_3d.clear()
        ax_3d.set_xlim(-1.5, 1.5); ax_3d.set_ylim(-1.5, 1.5); ax_3d.set_zlim(-1.5, 1.5)
        ax_3d.set_xlabel('X (Axis)'); ax_3d.set_ylabel('Y (Width)'); ax_3d.set_zlabel('Z (Height)')
        
        ax_3d.plot([-2, 2], [0, 0], [0, 0], 'r--', linewidth=2)
        verts = get_cube_verts(np.array([0,0,0]), L_x, L_y, Thickness, state['angle'])
        poly = Poly3DCollection(verts, facecolors='cyan', edgecolors='k', alpha=0.2)
        ax_3d.add_collection3d(poly)
        ax_3d.scatter(pos_load_global[0], pos_load_global[1], pos_load_global[2], color='red', s=150)
        ax_3d.scatter(pos_table_global[0], pos_table_global[1], pos_table_global[2], color='blue', marker='x', s=100)
        
        fig.canvas.draw_idle()

    # --- Interaction Handlers ---
    def on_click_map(event):
        if event.inaxes == ax_top:
            state['load_x'] = event.xdata
            state['load_y'] = event.ydata
            update_viz()

    # Slider Updates
    def update_angle_slider(val):
        state['angle'] = slider_angle.val
        box_angle.set_val(f"{val:.1f}")
        update_viz()
        
    def update_offset_slider(val):
        state['table_offset_y'] = slider_offset.val
        is_imp = (state['units'] == 'Imperial')
        factor = 39.37 if is_imp else 1.0
        disp_val = val * factor
        box_offset.set_val(f"{disp_val:.2f}") 
        update_viz()

    # Text Box Updates
    def submit_angle(text):
        try:
            val = float(text)
            val = max(0, min(360, val))
            state['angle'] = val
            slider_angle.eventson = False
            slider_angle.set_val(val)
            slider_angle.eventson = True
            update_viz()
        except ValueError:
            pass

    def submit_offset(text):
        try:
            val = float(text)
            is_imp = (state['units'] == 'Imperial')
            if is_imp:
                val_m = val / 39.37
            else:
                val_m = val
            val_m = max(-0.2, min(0.2, val_m))
            state['table_offset_y'] = val_m
            slider_offset.eventson = False
            slider_offset.set_val(val_m)
            slider_offset.eventson = True
            update_viz()
        except ValueError:
            pass

    def change_units(label):
        state['units'] = label
        update_offset_slider(state['table_offset_y'])
        update_viz()

    fig.canvas.mpl_connect('button_press_event', on_click_map)
    
    # --- Widgets ---
    
    # 1. Rotation Controls
    slider_angle = Slider(ax_sl_ang, 'Rotation (deg)', 0, 360, valinit=0)
    box_angle = TextBox(ax_box_ang, '', initial="0.0")
    
    # 2. Unbalance Controls
    slider_offset = Slider(ax_sl_off, 'Unbalance (m)', -0.2, 0.2, valinit=0.0)
    box_offset = TextBox(ax_box_off, '', initial="0.0")
    
    # 3. Units - Default Metric now
    radio = RadioButtons(ax_radio, ('Imperial', 'Metric'), active=1)
    
    # Connect Events
    slider_angle.on_changed(update_angle_slider)
    box_angle.on_submit(submit_angle)
    
    slider_offset.on_changed(update_offset_slider)
    box_offset.on_submit(submit_offset)
    
    radio.on_clicked(change_units)
    
    # Init
    update_viz()
    plt.show()

if __name__ == "__main__":
    final_physics_sim_inertia()