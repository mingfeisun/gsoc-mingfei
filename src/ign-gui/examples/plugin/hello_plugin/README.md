## Build

    mkdir build
    cd build
    cmake ..
    make

## Run

Standalone

    cd build
    export IGN_GUI_PLUGIN_PATH=`pwd`; ign gui -s HelloPlugin

Or open an empty window and insert from menu

    cd build
    export IGN_GUI_PLUGIN_PATH=`pwd`; ign gui
    # Choose HelloPlugin from menu
