import yaml
from lxml import etree
from pathlib import Path
import math
import pandas as pd
import numpy as np

# Dosya yolları
sdf_pose_file = Path("~/PX4-Autopilot/Tools/simulation/gz/models/x500_base_custom/model.sdf").expanduser()
sdf_plugin_file = Path("~/PX4-Autopilot/Tools/simulation/gz/models/x500_custom/model.sdf").expanduser()
yaml_file = Path("~/PX4-Autopilot/Tools/simulation/gz/models/x500_base_custom/scripts/motors.yaml").expanduser()

# Eğer CSV datası varsa yolu buraya yaz, yoksa None bırak
# csv_file = None
csv_file = Path("~/PX4-Autopilot/Tools/simulation/gz/models/x500_base_custom/scripts/x11.csv").expanduser()

# YAML oku
with open(yaml_file) as f:
    cfg = yaml.safe_load(f)

# -------------------------------------------------------------
# Yardımcı: Rotor inertia hesapla
# -------------------------------------------------------------
def compute_rotor_inertia(motor_mass, prop_radius):
    # Motoru silindir kabul et (r=0.03 m, h=0.05 m)
    r, h = 0.03, 0.05
    I_motor_xx = (1/12) * motor_mass * (3*r**2 + h**2)
    I_motor_yy = I_motor_xx
    I_motor_zz = 0.5 * motor_mass * r**2

    # Propelleri ince disk kabul et (motor kütlesinin %10'u kadar)
    m_prop = 0.1 * motor_mass
    I_prop_xx = 0.5 * m_prop * prop_radius**2
    I_prop_yy = I_prop_xx
    I_prop_zz = m_prop * prop_radius**2

    return (I_motor_xx + I_prop_xx,
            I_motor_yy + I_prop_yy,
            I_motor_zz + I_prop_zz)

# -------------------------------------------------------------
# 1) Pose + mass bilgilerini base SDF dosyasına yaz
# -------------------------------------------------------------
pose_tree = etree.parse(str(sdf_pose_file))
pose_root = pose_tree.getroot()

# 5010_motor_base_0'dan 5010_motor_base_3'e kadar olan motorlar için pose güncellemesi
for motor_index in range(4):  # motor_base_0'dan motor_base_3'e kadar
    motor_name = f"5010_motor_base_{motor_index}"
    
    # Pose güncelle
    link_pose = pose_root.find(f".//link[@name='{motor_name}']/pose")
    if link_pose is not None:
        # x ve y koordinatlarını değiştirmek için motor bilgilerini kullan
        motor = cfg["motors"][motor_index]  # motor bilgilerini almak
        new_x = motor["pose"][0]  # Yeni x koordinatını al
        new_y = motor["pose"][1]  # Yeni y koordinatını al
        new_z = link_pose.text.split()[2]  # z koordinatını aynı bırak
        new_pose = f"{new_x} {new_y} {new_z}"

        link_pose.text = new_pose
        print(f"✅ {motor_name} pose -> {new_pose}")

# Base link mass güncelle
base_mass = float(cfg.get("base_mass", 2.0))
base_mass_elem = pose_root.find(".//link[@name='base_link']/inertial/mass")
if base_mass_elem is not None:
    base_mass_elem.text = f"{base_mass:.3f}"
    print(f"✅ base_link mass = {base_mass} kg")

pose_tree.write(str(sdf_pose_file), pretty_print=True, xml_declaration=True, encoding="UTF-8")
print(f"\n✅ Pose + mass güncellemeleri {sdf_pose_file} dosyasına yazıldı.")

# -------------------------------------------------------------
# 2) Motor parametrelerini hesapla ve plugin SDF dosyasına yaz
# -------------------------------------------------------------
plugin_tree = etree.parse(str(sdf_plugin_file))
plugin_root = plugin_tree.getroot()

rho = 1.2041  # hava yoğunluğu (kg/m³ @ 20°C)

if csv_file and csv_file.exists():
    print(f"\n📂 CSV datası bulundu: {csv_file}")
    df = pd.read_csv(csv_file)

    # Thrust (N), omega (rad/s), torque (Nm)
    thrust_N = df["Thrust(g)"].astype(float).values * 9.81 / 1000.0
    rpm = df["Speed(RPM)"].astype(float).values
    omega = rpm * 2 * math.pi / 60.0
    torque = df["Torque(N·m)"].astype(float).values

    # Fit: motor constant
    coeff_motor = np.polyfit(omega**2, thrust_N, 1)
    motor_constant = coeff_motor[0]

    # Fit: moment constant
    coeff_moment = np.polyfit(thrust_N, torque, 1)
    moment_constant = coeff_moment[0]

    # Max rot velocity
    omega_max = omega.max()

    print(f"🔹 Ölçümden hesaplanan parametreler:")
    print(f"   MaxRotVelocity = {omega_max:.2f} rad/s")
    print(f"   MotorConstant  = {motor_constant:.4e}")
    print(f"   MomentConstant = {moment_constant:.4f}")

    for motor in cfg["motors"]:
        mid = motor["id"]
        plugin = plugin_root.find(f".//plugin[motorNumber='{mid}']")
        if plugin is not None:
            plugin.find("maxRotVelocity").text = f"{omega_max:.4f}"
            plugin.find("motorConstant").text = f"{motor_constant:.6e}"
            plugin.find("momentConstant").text = f"{moment_constant:.6f}"
            plugin.find("rotorDragCoefficient").text = f"{1e-5:.6e}"
            plugin.find("rollingMomentCoefficient").text = f"{1e-6:.1e}"
            plugin.find("timeConstantUp").text = f"{0.05:.3f}"
            plugin.find("timeConstantDown").text = f"{0.1:.3f}"
            print(f"✅ rotor_{mid} plugin CSV verileriyle güncellendi.")

else:
    print("\n⚠️ CSV datası bulunamadı, YAML değerleri ile hesap yapılacak.")
    for motor in cfg["motors"]:
        Kv = float(motor["Kv"])
        Vmax = float(motor["voltage"])
        eta = float(motor.get("efficiency", 0.9))
        Tmax = float(motor["max_thrust"])
        D = float(motor["prop_diameter"])
        mid = motor["id"]

        # Hesaplamalar (teorik fallback)
        omega_max = Kv * Vmax * eta * (2 * math.pi / 60) * 0.7
        k_t = Tmax / (omega_max**2)
        k_m = (0.5 * 60) / (2 * math.pi * Kv)

        Cd = Tmax / (rho * ((Kv * Vmax * eta / 60) ** 2) * (D**4))
        Cr = 1e-6
        tau_up = 0.05
        tau_down = 0.1

        plugin = plugin_root.find(f".//plugin[motorNumber='{mid}']")
        if plugin is not None:
            plugin.find("maxRotVelocity").text = f"{omega_max:.4f}"
            plugin.find("motorConstant").text = f"{k_t:.6e}"
            plugin.find("momentConstant").text = f"{k_m:.6f}"
            plugin.find("rotorDragCoefficient").text = f"{Cd:.6e}"
            plugin.find("rollingMomentCoefficient").text = f"{Cr:.1e}"
            plugin.find("timeConstantUp").text = f"{tau_up:.3f}"
            plugin.find("timeConstantDown").text = f"{tau_down:.3f}"
            print(f"✅ rotor_{mid} plugin YAML verileriyle güncellendi.")

plugin_tree.write(str(sdf_plugin_file), pretty_print=True, xml_declaration=True, encoding="UTF-8")
print(f"\n✅ Motor parametreleri {sdf_plugin_file} dosyasına yazıldı.")
