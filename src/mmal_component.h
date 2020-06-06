#include <interface/mmal/mmal_types.h>

#include "mmal_port.h"

#ifndef _INCL_MMAL_COMPONENT
#define _INCL_MMAL_COMPONENT

class MMAL_Component {
    private:
        MMAL_COMPONENT_T        _component;

        MMAL_Port               _controlPort;
        MMAL_Port               _inputPort;
        MMAL_Port               _outputPort;
        MMAL_Port               _clockPort;

        uint32_t                id;

    public:    
};

#endif
