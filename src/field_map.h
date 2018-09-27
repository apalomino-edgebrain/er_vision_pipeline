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

	class FieldManager
	{


	};
}

#endif