// ----------------------------------------------------------------
// ObjHandler class to handle import and export of meshes
// for the Advanced Graphics course of the Master Game Dev
// ----------------------------------------------------------------

#ifndef ADG_OBJ_HANDLER_H
#define ADG_OBJ_HANDLER_H

#include "Mesh.h"

namespace adg {

    class ObjHandler {
    public:
        static bool load(const std::string& file_path, Mesh& outMesh);
        static bool save(const std::string& file_path, const std::string& file_name, const Mesh& mesh);
    };
}


#endif // !ADG_OBJ_HANDLER_H