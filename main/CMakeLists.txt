set(COMPONENT_SRCS  "hello_world_main.c"
                    "http_server.c"
                    "json.c"
                    "wifi_manager.c")

set(COMPONENT_EMBED_FILES "assets/style.css"
                          "assets/jquery.gz"
                          "assets/code.js"
                          "assets/index.html")

set(COMPONENT_ADD_INCLUDEDIRS ".")

register_component()