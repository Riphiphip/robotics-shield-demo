cmake_minimum_required(VERSION 3.20.0)

if(CONFIG_LS7184N)
    target_sources(app PRIVATE
        ls7184n.c
    )
endif()
