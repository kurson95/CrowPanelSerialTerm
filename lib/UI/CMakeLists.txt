SET(SOURCES ui_serialTerm.c
    ui_WifiSettings.c
    ui_theme_manager.c
    ui_themes.c
    ui.c
    ui_comp_hook.c
    ui_helpers.c
    ui_events.c
    ui_img_backblue800x480_png.c
    ui_img_1149142576.c
    ui_img_1659073291.c
    ui_img_wifiapp_png.c
    ui_img_548092219.c)

add_library(ui ${SOURCES})
