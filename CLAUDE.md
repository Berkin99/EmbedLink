# Embedlink MCP Claude Information

1. Read README.md for Embedlink repo information
2. Use embedlink_mcp for build, flash and serial debug. This functions work on base stm32 project folder.
3. Embedlink is configured for serialPrint, serialScan for serial debugging with host computer. 
4. Embedlink folder (This folder)   : "{$ProjectWorkspace}/Core/Src/Embedlink"
5. Build folder                     : "{$ProjectWorkspace}/Release/*  or  "{$ProjectWorkspace}/Debug/*
6. Serial port                      : Use embedlink_mcp serial_list command you will see related com port.
7. Serial port baudrate             : 115200

Claude work pipeline :
    get prompt
    do:
        develop the code
        add serial prints for debugging
        build
        flash & run
        serial debug
    while (code is not working)