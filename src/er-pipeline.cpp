//#############################################################################
                                 /*-:::--.`
                            -+shmMMNmmddmmNMNmho/.
                 `yyo++osddMms:                  `/yNNy-
              yo    +mMy:                       `./dMMdyssssso-
              oy  -dMy.                     `-+ssso:.`:mMy`   .ys
                ho+MN:                  `:osso/.         oMm-   +h
                +Mmd-           `/syo-                     :MNhs`
                `NM-.hs`      :syo:                          sMh
                oMh   :ho``/yy+.                             `MM.
                hM+    `yNN/`                                 dM+
                dM/  -sy/`/ho`                                hMo
                hMo/ho.     :yy-                             dM/
            :dNM/             :yy:                         yMy
            sy`:MN.              `+ys-                     +Mm`
            oy`   :NM+                  .+ys/`           `hMd.ys
            /sssssyNMm:                   `:sys:`     `oNN+   m-
                        .sNMh+.                   `:sNMdyysssssy:
                        -odMNhs+:-.`    `.-/oydMNh+.
                            `-+shdNMMMMMMMNmdyo/.
                                    `````*/
//#############################################################################
// Pipeline to process our PointCloud data
//#############################################################################
//
// See er-pipeline.h to get more information on the functionality and structure
//

#include "er-pipeline.h"

//-----------------------------------------------------------------------------
// STD
//-----------------------------------------------------------------------------

#include <string>
using namespace std;

//-----------------------------------------------------------------------------
// BOOST System
//-----------------------------------------------------------------------------

#include <boost/filesystem.hpp>
using namespace boost::filesystem;


er::pipeline::pipeline()
{
    printf("+ Initialize pipeline\n");
}

er::pipeline::~pipeline()
{

}

void er::pipeline::initialize_folder(std::string folder_path)
{
    path data_path(folder_path);
    if (!is_directory(data_path)) {
        data_path = data_path.parent_path();

        if (!is_directory(data_path)) {
            printf("! Error: Not a directory [%s]\n", folder_path.c_str());
        }
    } else {
        printf("+ Processing [%s]\n", folder_path.c_str());
    }

    for (directory_iterator itr(data_path); itr != directory_iterator(); ++itr) {
        // display filename only
        cout << itr->path().filename() << ' ';
        if (is_regular_file(itr->status()))
            cout << " [" << file_size(itr->path()) << ']';

        cout << '\n';
    }
}

void er::pipeline::process_frame(pcl_ptr cloud)
{

}