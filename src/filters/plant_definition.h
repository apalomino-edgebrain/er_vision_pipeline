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
// Just a simple plant definition to expand on it
//#############################################################################

#ifndef plant_definition_filter_H_
#define plant_definition_filter_H_

namespace er {
	// Temporary plant type
	enum class plant_types
	{
		crop, weed
	};

	class plant_definition
	{
	public:
		plant_definition();

		plant_types type;

		// ---- 3D Space ----
		float pos_x, pos_y;
		float width, height;

		// ---- 2D Images ----
		// We resolve the U,V coordinates and we extract the plant as a 2D image
		// without background by asking the realsense APIs
		void *ptr_texture;

		float view_radius;
		float view_x, view_y;
		float view_width, view_height;
	};
}

#endif