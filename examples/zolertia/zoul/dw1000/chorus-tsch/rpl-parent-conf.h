/* Ensure that the NODEID macro is correctly passed by the Makefile.
   That's not the case for each target platform. */
#ifndef NODEID
#  error "Node ID not defined. Perhaps you need to tweak target Makefile."
#endif /* NODEID */


#define RPL_PARENT_SELECT_ID 0x1
