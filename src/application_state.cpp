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
// Application global state
//#############################################################################

#include <string>
#include <iostream>

#include "json/json.hpp"
using namespace nlohmann;

#include "application_state.h"
#include "util/er-logging.h"

#include <boost/filesystem.hpp>
using namespace boost::filesystem;

er::app_state::app_state()
{
    printf("+ Initialize pipeline\n");
	load_configuration(DEFAULT_CONFIG);
}

er::app_state::~app_state()
{

}

void er::app_state::load_configuration (std::string json_config)
{
	printf_("Reading config", json_config.c_str());
	std::ifstream i(json_config.c_str());
	i >> config;

	printf_h2("Config");

	std::cout << std::setw(4) << config << std::endl;
}

void er::app_state::save_configuration()
{

	std::cout << "Save" << std::endl;
}

void er::app_state::set_current_file(std::string filepath_playback)
{
	path data_path(filepath_playback);
	if (!is_directory(data_path)) {
		capture_bag = filepath_playback;
		capture_folder = data_path.parent_path().string();
	} else {
		capture_bag = filepath_playback + "/capture.bag";
		capture_folder = filepath_playback;
	}

	invalidate_playback = true;
	save_configuration();
}
