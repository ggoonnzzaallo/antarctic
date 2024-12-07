from urdf2mjcf import urdf2mjcf

# Convert URDF to MJCF
mjcf_xml = urdf2mjcf("first_import/robot.urdf", "output.xml")

print(mjcf_xml)