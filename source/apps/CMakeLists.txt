# Fetch lenny-gui-opengl
fetch_lenny_repository(gui-opengl master)

# Fetch lenny-bots
fetch_lenny_repository(bots master)

# Add subdirectories
add_subdirectory(CollisionApp)
add_subdirectory(MAMPApp)
add_subdirectory(SAMPApp)
