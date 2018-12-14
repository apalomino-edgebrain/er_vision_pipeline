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
// Filter to extract the ground information
//#############################################################################

#ifndef plant_extraction_filter_H_
#define plant_extraction_filter_H_

#include "../er-pipeline.h"
#include "../algebra/plane.h"

namespace er {
	class plants_filter: public process_unit
	{
	public:
		pcl_ptr cloud_render;

		ground_filter *grnd_filter;

		plants_filter(): grnd_filter { nullptr } {};
		~plants_filter() {};

		void set_ground_filter(ground_filter *grnd_filter_);
		bool process() override;
		void invalidate_view() override;
		void render_ui() override;
	};
}

#endif