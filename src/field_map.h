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
// Field map in 3D space with time and data
//#############################################################################

#ifndef FIELD_MAP_H_
#define FIELD_MAP_H_

#include <vector>

namespace er {
	class geo_frame
	{
		uint32_t id;

		// Detected max and minimum lat long
		float lat_start, long_start;
		float lat_end, long_end;
	};

	class field_map : public geo_frame
	{
		std::vector<int> Row;
	};

	class field_row : public geo_frame
	{
		std::vector<int> plant_ids;
	};

	class field_plant : public geo_frame
	{
		// Ids for frames which contain this object
		std::vector<int> plant_frames;
		uint32_t row_id;
	};

	class field_manager
	{


	};
}

#endif