/* Ensure that the NODEID macro is correctly passed by the Makefile.
   That's not the case for each target platform. */
#ifndef NODEID
#  error "Node ID not defined. Perhaps you need to tweak target Makefile."
#endif /* NODEID */

#if NODEID == 0x01
    #define RPL_PARENT_SELECT_ID 0x1
#elif NODEID == 0x02
    #define RPL_PARENT_SELECT_ID 0x1
#elif NODEID == 0x03
    #define RPL_PARENT_SELECT_ID 0x1
#elif NODEID == 0x04
    #define RPL_PARENT_SELECT_ID 0x1
#elif NODEID == 0x05
    #define RPL_PARENT_SELECT_ID 0x6
#elif NODEID == 0x06
    #define RPL_PARENT_SELECT_ID 0x1
#elif NODEID == 0x07
    #define RPL_PARENT_SELECT_ID 0x3
#elif NODEID == 0x08
    #define RPL_PARENT_SELECT_ID 0x6
#elif NODEID == 0x09
    #define RPL_PARENT_SELECT_ID 0x2
#elif NODEID == 0x0A
    #define RPL_PARENT_SELECT_ID 0x1
#elif NODEID == 0x0B
    #define RPL_PARENT_SELECT_ID 0xa
#elif NODEID == 0x0C
    #define RPL_PARENT_SELECT_ID 0xa
#elif NODEID == 0x0D
    #define RPL_PARENT_SELECT_ID 0x10
#elif NODEID == 0x0E
    #define RPL_PARENT_SELECT_ID 0x10
#elif NODEID == 0x0F
    #define RPL_PARENT_SELECT_ID 0x10
#elif NODEID == 0x10
    #define RPL_PARENT_SELECT_ID 0x7
#elif NODEID == 0x11
    #define RPL_PARENT_SELECT_ID 0x10
#elif NODEID == 0x12
    #define RPL_PARENT_SELECT_ID 0x3
#endif /* NODEID */
