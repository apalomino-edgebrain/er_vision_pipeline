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
// Imgui for the visualizer
//#############################################################################

#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/thread/lockable_adapter.hpp>
#include <mutex>
#include <iostream>
#include <random>

#include <igl/readOFF.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>

#include <igl/jet.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

#include "er-pipeline.h"

#include "application_state.h"

igl::opengl::glfw::imgui::ImGuiMenu menu;

using namespace er;

// Shared state of InputText(), passed as an argument to your callback when a ImGuiInputTextFlags_Callback* flag is used.
// The callback function should return 0 by default.
// Special processing:
// - ImGuiInputTextFlags_CallbackCharFilter:  return 1 if the character is not allowed. You may also set 'EventChar=0' as any character replacement are allowed.
// - ImGuiInputTextFlags_CallbackResize:      notified by InputText() when the string is resized. BufTextLen is set to the new desired string length so you can update the string size on your side of the fence. You can also replace Buf pointer if your underlying data is reallocated. No need to initialize new characters or zero-terminator as InputText will do it right after the resize callback.
struct ImGuiInputTextCallbackData
{
	ImGuiInputTextFlags EventFlag;      // One ImGuiInputTextFlags_Callback*    // Read-only
	ImGuiInputTextFlags Flags;          // What user passed to InputText()      // Read-only
	void*               UserData;       // What user passed to InputText()      // Read-only

										// Arguments for the different callback events
										// - To modify the text buffer in a callback, prefer using the InsertChars() / DeleteChars() function. InsertChars() will take care of calling the resize callback if necessary.
										// - If you know your edits are not going to resize the underlying buffer allocation, you may modify the contents of 'Buf[]' directly. You need to update 'BufTextLen' accordingly (0 <= BufTextLen < BufSize) and set 'BufDirty'' to true so InputText can update its internal state.
	ImWchar             EventChar;      // Character input                      // Read-write   // [CharFilter] Replace character or set to zero. return 1 is equivalent to setting EventChar=0;
	ImGuiKey            EventKey;       // Key pressed (Up/Down/TAB)            // Read-only    // [Completion,History]
	char*               Buf;            // Text buffer                          // Read-write   // [Resize] Can replace pointer / [Completion,History,Always] Only write to pointed data, don't replace the actual pointer!
	int                 BufTextLen;     // Text length in bytes                 // Read-write   // [Resize,Completion,History,Always] Exclude zero-terminator storage. In C land: == strlen(some_text), in C++ land: string.length()
	int                 BufSize;        // Buffer capacity in bytes             // Read-only    // [Resize,Completion,History,Always] Include zero-terminator storage. In C land == ARRAYSIZE(my_char_array), in C++ land: string.capacity()+1
	bool                BufDirty;       // Set if you modify Buf/BufTextLen!!   // Write        // [Completion,History,Always]
	int                 CursorPos;      //                                      // Read-write   // [Completion,History,Always]
	int                 SelectionStart; //                                      // Read-write   // [Completion,History,Always] == to SelectionEnd when no selection)
	int                 SelectionEnd;   //                                      // Read-write   // [Completion,History,Always]

										// Helper functions for text manipulation.
										// Use those function to benefit from the CallbackResize behaviors. Calling those function reset the selection.
	ImGuiInputTextCallbackData();
	IMGUI_API void      DeleteChars(int pos, int bytes_count);
	IMGUI_API void      InsertChars(int pos, const char* text, const char* text_end = NULL);
	bool                HasSelection() const { return SelectionStart != SelectionEnd; }
};

//-----------------------------------------------------------------------------
// [SECTION] Example App: Debug Console / ShowExampleAppConsole()
//-----------------------------------------------------------------------------

// Demonstrate creating a simple console window, with scrolling, filtering, completion and history.
// For the console example, here we are using a more C++ like approach of declaring a class to hold the data and the functions.
struct ExampleAppConsole
{
	char                  InputBuf[256];
	ImVector<char*>       Items;
	bool                  ScrollToBottom;
	ImVector<char*>       History;
	int                   HistoryPos;    // -1: new line, 0..History.Size-1 browsing history.
	ImVector<const char*> Commands;

	ExampleAppConsole()
	{
		ClearLog();
		memset(InputBuf, 0, sizeof(InputBuf));
		HistoryPos = -1;
		Commands.push_back("HELP");
		Commands.push_back("HISTORY");
		Commands.push_back("CLEAR");
		Commands.push_back("CLASSIFY");  // "classify" is only here to provide an example of "C"+[tab] completing to "CL" and displaying matches.
		AddLog("Welcome to Earth Rover processing");
	}
	~ExampleAppConsole()
	{
		ClearLog();
		for (int i = 0; i < History.Size; i++)
			free(History[i]);
	}

	// Portable helpers
	static int   Stricmp(const char* str1, const char* str2) { int d; while ((d = toupper(*str2) - toupper(*str1)) == 0 && *str1) { str1++; str2++; } return d; }
	static int   Strnicmp(const char* str1, const char* str2, int n) { int d = 0; while (n > 0 && (d = toupper(*str2) - toupper(*str1)) == 0 && *str1) { str1++; str2++; n--; } return d; }
	static char* Strdup(const char *str) { size_t len = strlen(str) + 1; void* buff = malloc(len); return (char*) memcpy(buff, (const void*) str, len); }
	static void  Strtrim(char* str) { char* str_end = str + strlen(str); while (str_end > str && str_end[-1] == ' ') str_end--; *str_end = 0; }

	void    ClearLog()
	{
		for (int i = 0; i < Items.Size; i++)
			free(Items[i]);
		Items.clear();
		ScrollToBottom = true;
	}

	void    AddLog(const char* fmt, ...) IM_FMTARGS(2)
	{
		// FIXME-OPT
		char buf[1024];
		va_list args;
		va_start(args, fmt);
		vsnprintf(buf, IM_ARRAYSIZE(buf), fmt, args);
		buf[IM_ARRAYSIZE(buf) - 1] = 0;
		va_end(args);
		Items.push_back(Strdup(buf));
		ScrollToBottom = true;
	}

	void    Draw(const char* title, bool* p_open)
	{
		ImGui::SetNextWindowSize(ImVec2(520, 600), ImGuiCond_FirstUseEver);
		if (!ImGui::Begin(title, p_open)) {
			ImGui::End();
			return;
		}

		// As a specific feature guaranteed by the library, after calling Begin() the last Item represent the title bar. So e.g. IsItemHovered() will return true when hovering the title bar.
		// Here we create a context menu only available from the title bar.
		if (ImGui::BeginPopupContextItem()) {
			if (ImGui::MenuItem("Close Console"))
				*p_open = false;
			ImGui::EndPopup();
		}

		//ImGui::TextWrapped("This example implements a console with basic coloring, completion and history. A more elaborate implementation may want to store entries along with extra data such as timestamp, emitter, etc.");
		//ImGui::TextWrapped("Enter 'HELP' for help, press TAB to use text completion.");

		// TODO: display items starting from the bottom

		//if (ImGui::SmallButton("Add Dummy Text")) { AddLog("%d some text", Items.Size); AddLog("some more text"); AddLog("display very important message here!"); } ImGui::SameLine();
		//if (ImGui::SmallButton("Add Dummy Error")) { AddLog("[error] something went wrong"); } ImGui::SameLine();
		if (ImGui::SmallButton("Clear")) { ClearLog(); } ImGui::SameLine();
		bool copy_to_clipboard = ImGui::SmallButton("Copy"); ImGui::SameLine();
		if (ImGui::SmallButton("Scroll to bottom")) ScrollToBottom = true;
		//static float t = 0.0f; if (ImGui::GetTime() - t > 0.02f) { t = ImGui::GetTime(); AddLog("Spam %f", t); }

		ImGui::Separator();

		ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0, 0));
		static ImGuiTextFilter filter;
		filter.Draw("Filter (\"incl,-excl\") (\"error\")", 180);
		ImGui::PopStyleVar();
		ImGui::Separator();

		const float footer_height_to_reserve = ImGui::GetStyle().ItemSpacing.y + ImGui::GetFrameHeightWithSpacing(); // 1 separator, 1 input text
		ImGui::BeginChild("ScrollingRegion", ImVec2(0, -footer_height_to_reserve), false, ImGuiWindowFlags_HorizontalScrollbar); // Leave room for 1 separator + 1 InputText
		if (ImGui::BeginPopupContextWindow()) {
			if (ImGui::Selectable("Clear")) ClearLog();
			ImGui::EndPopup();
		}

		// Display every line as a separate entry so we can change their color or add custom widgets. If you only want raw text you can use ImGui::TextUnformatted(log.begin(), log.end());
		// NB- if you have thousands of entries this approach may be too inefficient and may require user-side clipping to only process visible items.
		// You can seek and display only the lines that are visible using the ImGuiListClipper helper, if your elements are evenly spaced and you have cheap random access to the elements.
		// To use the clipper we could replace the 'for (int i = 0; i < Items.Size; i++)' loop with:
		//     ImGuiListClipper clipper(Items.Size);
		//     while (clipper.Step())
		//         for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; i++)
		// However, note that you can not use this code as is if a filter is active because it breaks the 'cheap random-access' property. We would need random-access on the post-filtered list.
		// A typical application wanting coarse clipping and filtering may want to pre-compute an array of indices that passed the filtering test, recomputing this array when user changes the filter,
		// and appending newly elements as they are inserted. This is left as a task to the user until we can manage to improve this example code!
		// If your items are of variable size you may want to implement code similar to what ImGuiListClipper does. Or split your data into fixed height items to allow random-seeking into your list.
		ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(4, 1)); // Tighten spacing
		if (copy_to_clipboard)
			ImGui::LogToClipboard();
		ImVec4 col_default_text = ImGui::GetStyleColorVec4(ImGuiCol_Text);
		for (int i = 0; i < Items.Size; i++) {
			const char* item = Items[i];
			if (!filter.PassFilter(item))
				continue;
			ImVec4 col = col_default_text;
			if (strstr(item, "[error]")) col = ImColor(1.0f, 0.4f, 0.4f, 1.0f);
			else if (strncmp(item, "# ", 2) == 0) col = ImColor(1.0f, 0.78f, 0.58f, 1.0f);
			ImGui::PushStyleColor(ImGuiCol_Text, col);
			ImGui::TextUnformatted(item);
			ImGui::PopStyleColor();
		}
		if (copy_to_clipboard)
			ImGui::LogFinish();
		if (ScrollToBottom)
			ImGui::SetScrollHere(1.0f);
		ScrollToBottom = false;
		ImGui::PopStyleVar();
		ImGui::EndChild();
		ImGui::Separator();

		// Command-line
		bool reclaim_focus = false;

		/*
		if (ImGui::InputText("Input", InputBuf, IM_ARRAYSIZE(InputBuf), ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_CallbackCompletion | ImGuiInputTextFlags_CallbackHistory, &TextEditCallbackStub, (void*)this)) {
			char* s = InputBuf;
			Strtrim(s);
			if (s[0])
				ExecCommand(s);
			strcpy(s, "");
			reclaim_focus = true;
		}
		*/

		// Auto-focus on window apparition
		ImGui::SetItemDefaultFocus();
		if (reclaim_focus)
			ImGui::SetKeyboardFocusHere(-1); // Auto focus previous widget

		ImGui::End();
	}

	void    ExecCommand(const char* command_line)
	{
		AddLog("# %s\n", command_line);

		// Insert into history. First find match and delete it so it can be pushed to the back. This isn't trying to be smart or optimal.
		HistoryPos = -1;
		for (int i = History.Size - 1; i >= 0; i--)
			if (Stricmp(History[i], command_line) == 0) {
				free(History[i]);
				History.erase(History.begin() + i);
				break;
			}
		History.push_back(Strdup(command_line));

		// Process command
		if (Stricmp(command_line, "CLEAR") == 0) {
			ClearLog();
		} else if (Stricmp(command_line, "HELP") == 0) {
			AddLog("Commands:");
			for (int i = 0; i < Commands.Size; i++)
				AddLog("- %s", Commands[i]);
		} else if (Stricmp(command_line, "HISTORY") == 0) {
			int first = History.Size - 10;
			for (int i = first > 0 ? first : 0; i < History.Size; i++)
				AddLog("%3d: %s\n", i, History[i]);
		} else {
			AddLog("Unknown command: '%s'\n", command_line);
		}
	}

	static int TextEditCallbackStub(ImGuiInputTextCallbackData* data) // In C++11 you are better off using lambdas for this sort of forwarding callbacks
	{
		ExampleAppConsole* console = (ExampleAppConsole*) data->UserData;
		return console->TextEditCallback(data);
	}

	int     TextEditCallback(ImGuiInputTextCallbackData* data)
	{
		//AddLog("cursor: %d, selection: %d-%d", data->CursorPos, data->SelectionStart, data->SelectionEnd);
		switch (data->EventFlag) {
			case ImGuiInputTextFlags_CallbackCompletion:
			{
				// Example of TEXT COMPLETION

				// Locate beginning of current word
				const char* word_end = data->Buf + data->CursorPos;
				const char* word_start = word_end;
				while (word_start > data->Buf) {
					const char c = word_start[-1];
					if (c == ' ' || c == '\t' || c == ',' || c == ';')
						break;
					word_start--;
				}

				// Build a list of candidates
				ImVector<const char*> candidates;
				for (int i = 0; i < Commands.Size; i++)
					if (Strnicmp(Commands[i], word_start, (int) (word_end - word_start)) == 0)
						candidates.push_back(Commands[i]);

				if (candidates.Size == 0) {
					// No match
					AddLog("No match for \"%.*s\"!\n", (int) (word_end - word_start), word_start);
				} else if (candidates.Size == 1) {
					// Single match. Delete the beginning of the word and replace it entirely so we've got nice casing
					data->DeleteChars((int) (word_start - data->Buf), (int) (word_end - word_start));
					data->InsertChars(data->CursorPos, candidates[0]);
					data->InsertChars(data->CursorPos, " ");
				} else {
					// Multiple matches. Complete as much as we can, so inputing "C" will complete to "CL" and display "CLEAR" and "CLASSIFY"
					int match_len = (int) (word_end - word_start);
					for (;;) {
						int c = 0;
						bool all_candidates_matches = true;
						for (int i = 0; i < candidates.Size && all_candidates_matches; i++)
							if (i == 0)
								c = toupper(candidates[i][match_len]);
							else if (c == 0 || c != toupper(candidates[i][match_len]))
								all_candidates_matches = false;
						if (!all_candidates_matches)
							break;
						match_len++;
					}

					if (match_len > 0) {
						data->DeleteChars((int) (word_start - data->Buf), (int) (word_end - word_start));
						data->InsertChars(data->CursorPos, candidates[0], candidates[0] + match_len);
					}

					// List matches
					AddLog("Possible matches:\n");
					for (int i = 0; i < candidates.Size; i++)
						AddLog("- %s\n", candidates[i]);
				}

				break;
			}
			case ImGuiInputTextFlags_CallbackHistory:
			{
				// Example of HISTORY
				const int prev_history_pos = HistoryPos;
				if (data->EventKey == ImGuiKey_UpArrow) {
					if (HistoryPos == -1)
						HistoryPos = History.Size - 1;
					else if (HistoryPos > 0)
						HistoryPos--;
				} else if (data->EventKey == ImGuiKey_DownArrow) {
					if (HistoryPos != -1)
						if (++HistoryPos >= History.Size)
							HistoryPos = -1;
				}

				// A better implementation would preserve the data on the current input line along with cursor position.
				if (prev_history_pos != HistoryPos) {
					const char* history_str = (HistoryPos >= 0) ? History[HistoryPos] : "";
					data->DeleteChars(0, data->BufTextLen);
					data->InsertChars(0, history_str);
				}
			}
		}
		return 0;
	}
};

static void ShowExampleAppConsole(bool* p_open)
{
	if (!*p_open)
		return;

	static ExampleAppConsole console;
	console.Draw("Console", p_open);
}

//-----------------------------------------------------------------------------
// [SECTION] Example App: Debug Log / ShowExampleAppLog()
//-----------------------------------------------------------------------------

// Usage:
//  static ExampleAppLog my_log;
//  my_log.AddLog("Hello %d world\n", 123);
//  my_log.Draw("title");
struct ExampleAppLog
{
	ImGuiTextBuffer     Buf;
	ImGuiTextFilter     Filter;
	ImVector<int>       LineOffsets;        // Index to lines offset
	bool                ScrollToBottom;

	void    Clear() { Buf.clear(); LineOffsets.clear(); }

	void    AddLog(const char* fmt, ...) IM_FMTARGS(2)
	{
		int old_size = Buf.size();
		va_list args;
		va_start(args, fmt);
		Buf.appendfv(fmt, args);
		va_end(args);
		for (int new_size = Buf.size(); old_size < new_size; old_size++)
			if (Buf[old_size] == '\n')
				LineOffsets.push_back(old_size);
		ScrollToBottom = true;
	}

	void    Draw(const char* title, bool* p_open = NULL)
	{
		ImGui::SetNextWindowSize(ImVec2(500, 400), ImGuiCond_FirstUseEver);
		if (!ImGui::Begin(title, p_open)) {
			ImGui::End();
			return;
		}
		if (ImGui::Button("Clear")) Clear();
		ImGui::SameLine();
		bool copy = ImGui::Button("Copy");
		ImGui::SameLine();
		Filter.Draw("Filter", -100.0f);
		ImGui::Separator();
		ImGui::BeginChild("scrolling", ImVec2(0, 0), false, ImGuiWindowFlags_HorizontalScrollbar);
		if (copy) ImGui::LogToClipboard();

		if (Filter.IsActive()) {
			const char* buf_begin = Buf.begin();
			const char* line = buf_begin;
			for (int line_no = 0; line != NULL; line_no++) {
				const char* line_end = (line_no < LineOffsets.Size) ? buf_begin + LineOffsets[line_no] : NULL;
				if (Filter.PassFilter(line, line_end))
					ImGui::TextUnformatted(line, line_end);
				line = line_end && line_end[1] ? line_end + 1 : NULL;
			}
		} else {
			ImGui::TextUnformatted(Buf.begin());
		}

		if (ScrollToBottom)
			ImGui::SetScrollHere(1.0f);
		ScrollToBottom = false;
		ImGui::EndChild();
		ImGui::End();
	}
};

// Demonstrate creating a simple log window with basic filtering.
static void ShowExampleAppLog(bool* p_open)
{
	static ExampleAppLog log;

	if (!*p_open)
		return;

	/*
	// Demo: add random items (unless Ctrl is held)
	static double last_time = -1.0;
	double time = ImGui::GetTime();
	if (time - last_time >= 0.20f && !ImGui::GetIO().KeyCtrl) {
		const char* random_words[] = { "system", "info", "warning", "error", "fatal", "notice", "log" };
		log.AddLog("[%s] Hello, time is %.1f, frame count is %d\n", random_words[rand() % IM_ARRAYSIZE(random_words)], time, ImGui::GetFrameCount());
		last_time = time;
	}
	*/

	log.Draw("Log", p_open);
}

void initialize_visualizer_ui(igl::opengl::glfw::Viewer &viewer)
{
	// Attach a menu plugin
	viewer.plugins.push_back(&menu);

	// Customize the menu
	float floatVariable = 0.1f; // Shared between two menus

	// Add content to the default menu window
	menu.callback_draw_viewer_menu = [&] () {
		menu.draw_viewer_menu();

		/*
		// Add new group
		if (ImGui::CollapsingHeader("New Group", ImGuiTreeNodeFlags_DefaultOpen)) {
			// Expose variable directly ...
			ImGui::InputFloat("float", &floatVariable, 0, 0, 3);

			// ... or using a custom callback
			static bool boolVariable = true;
			if (ImGui::Checkbox("bool", &boolVariable)) {
				// do something
				std::cout << "boolVariable: " << std::boolalpha << boolVariable << std::endl;
			}

			// Expose an enumeration type
			enum Orientation { Up = 0, Down, Left, Right };
			static Orientation dir = Up;
			ImGui::Combo("Direction", (int *) (&dir), "Up\0Down\0Left\0Right\0\0");

			// We can also use a std::vector<std::string> defined dynamically
			static int num_choices = 3;
			static std::vector<std::string> choices;
			static int idx_choice = 0;
			if (ImGui::InputInt("Num letters", &num_choices)) {
				num_choices = std::max(1, std::min(26, num_choices));
			}
			if (num_choices != (int) choices.size()) {
				choices.resize(num_choices);
				for (int i = 0; i < num_choices; ++i)
					choices[i] = std::string(1, 'A' + i);
				if (idx_choice >= num_choices)
					idx_choice = num_choices - 1;
			}
			ImGui::Combo("Letter", &idx_choice, choices);

			// Add a button
			if (ImGui::Button("Print Hello", ImVec2(-1, 0))) {
				std::cout << "Hello\n";
			}
		}
		*/
	};

	menu.callback_draw_custom_window = [&] () {

		ShowExampleAppConsole(&app_state::get().show_app_console);
		ShowExampleAppLog(&app_state::get().show_app_log);

		static const int flags = ImGuiWindowFlags_AlwaysAutoResize;
		{
			ImGuiIO& io = ImGui::GetIO();

			{
				ImGui::Begin("System", nullptr, flags | ImGuiWindowFlags_MenuBar);
				if (ImGui::BeginMenuBar()) {
					if (ImGui::BeginMenu("Menu")) {
						//if (ImGui::MenuItem("New")) {}
						if (ImGui::MenuItem("Open", "Ctrl+O")) {
							std::string fname = igl::file_dialog_open();

							if (fname.length() == 0)
								return;

							std::cout << "Load file " << fname << std::endl;
							app_state::get().set_current_file(fname);
						}
						if (ImGui::BeginMenu("Open Recent")) {
							//ImGui::MenuItem("capture.bag");
							ImGui::EndMenu();
						}
						//if (ImGui::MenuItem("Save", "Ctrl+S")) {}
						//if (ImGui::MenuItem("Save As..")) {}
						ImGui::Separator();
						ImGui::EndMenu();
					}
					if (ImGui::BeginMenu("Debug")) {
						ImGui::MenuItem("Console", NULL, &app_state::get().show_app_console);
						ImGui::MenuItem("Log", NULL, &app_state::get().show_app_log);
						ImGui::EndMenu();
					}

					ImGui::EndMenuBar();
				}

				ImGui::Text("Playback");
				ImGui::SameLine(70); ImGui::Text("%s",
					app_state::get().capture_folder.c_str());

				if (ImGui::CollapsingHeader("Capture", ImGuiTreeNodeFlags_DefaultOpen)) {
					float w = ImGui::GetContentRegionAvailWidth();
					float p = ImGui::GetStyle().FramePadding.x;
					if (ImGui::Button("Load##Analysis", ImVec2((w - p) / 2.f, 0))) {
						std::string fname = igl::file_dialog_open();

						if (fname.length() == 0)
							return;

						std::cout << "Load file " << fname << std::endl;
						app_state::get().set_current_file(fname);
					}
					ImGui::SameLine(0, p);
					if (ImGui::Button("Save##Analysis", ImVec2((w - p) / 2.f, 0))) {

					}
				}

				ImGui::End();
			}

			//ImGui::GetIO().IniFilename = "interface.ini";

			{
				ImGui::Begin("Raw RGBD View", nullptr, flags);

				ImGui::Text("Color Spaces");
				ImGui::Checkbox("Show nvdi", &app_state::get().bool_tint_nvdi);
				ImGui::SliderFloat("NVDI", &app_state::get().cur_nvdi,
					app_state::get().min_nvdi,
					app_state::get().max_nvdi);

				ImGui::Checkbox("Show ir", &app_state::get().bool_tint_ir);
				ImGui::SliderFloat("IR", &app_state::get().cur_ir, app_state::get().min_ir, app_state::get().max_ir);
				//ImGui::SliderFloat("HSV", &cur_nvdi, min_nvdi, max_nvdi);

				ImGui::Text("Clipping Z");
				ImGui::SliderFloat("Min Z", &app_state::get().cur_min_clip[2], app_state::get().min_clip[2], app_state::get().max_clip[2]);
				ImGui::SliderFloat("Max Z", &app_state::get().cur_max_clip[2], app_state::get().min_clip[2], app_state::get().max_clip[2]);

				ImGui::Text("Clipping Y");
				ImGui::SliderFloat("clipping Y", &app_state::get().cur_max_clip[1], app_state::get().min_clip[1], app_state::get().max_clip[1]);

				ImGui::Text("Point Scale");
				ImGui::SliderFloat("scale", &app_state::get().point_scale, 0.005f, 0.2f);

				if (ImGui::Checkbox("Show IR data", &app_state::get().show_ir_only_data)) {
					app_state::get().invalidate_ui = true;
					printf(" Show IR data \n");
				};

				ImGui::Separator();
				if (app_state::get().playing)
					ImGui::Checkbox("Playing", &app_state::get().playing);
				else
					ImGui::Checkbox("Pause", &app_state::get().playing);

				ImGui::Checkbox("Debug", &app_state::get().bool_debug_verbose);
				ImGui::End();
			}

			{
				ImGui::Begin("Debug Ground", nullptr, flags);
				ImGui::Text("Rotation");
				ImGui::Checkbox("Override", &app_state::get().bool_override_rotation);
				ImGui::SliderFloat("RotX", &app_state::get().rot_x, -2 * M_PI, 2 * M_PI);
				ImGui::SliderFloat("RotY", &app_state::get().rot_y, -2 * M_PI, 2 * M_PI);
				ImGui::SliderFloat("RotZ", &app_state::get().rot_z, -2 * M_PI, 2 * M_PI);

				ImGui::Text("Degree");
				ImGui::SameLine(100); ImGui::Text("%2.2f", app_state::get().rot_x * 360 / (2 * M_PI));
				ImGui::SameLine(150); ImGui::Text("%2.2f", app_state::get().rot_y * 360 / (2 * M_PI));
				ImGui::SameLine(200); ImGui::Text("%2.2f", app_state::get().rot_z * 360 / (2 * M_PI));

				ImGui::Checkbox("Traslate", &app_state::get().bool_traslate);
				ImGui::End();
			}
		}

		{
			ImGui::Begin("Analysis", &app_state::get().show_analysis, flags);

			if (ImGui::Checkbox("Show ground", &app_state::get().show_ground)) {
				app_state::get().invalidate_ui = true;
				printf(" Show ground\n");
			}

			if (ImGui::Checkbox("Show plants", &app_state::get().show_plants)) {
				app_state::get().invalidate_ui = true;
				printf(" Show plants\n");
			}

			if (ImGui::Checkbox("Extract plants", &app_state::get().bool_extract_plants)) {
				app_state::get().invalidate_ui = true;
				printf(" Extract plants\n");
			};

			ImGui::Separator();

			if (ImGui::Checkbox("BBX", &app_state::get().show_bbx)) {
				app_state::get().invalidate_ui = true;
				printf(" Color cluster \n");
			};

			if (ImGui::Checkbox("Raw Cloud", &app_state::get().bool_cloud_raw)) {
				app_state::get().invalidate_ui = true;
				printf(" Color cluster \n");
			};

			if (ImGui::Checkbox("Color cluster", &app_state::get().bool_color_cluster)) {
				app_state::get().invalidate_ui = true;
				printf(" Color cluster \n");
			};

			if (ImGui::Checkbox("Voxel Process", &app_state::get().bool_voxel_process)) {
				app_state::get().invalidate_ui = true;
				printf(" Voxel Process \n");
			};

			if (ImGui::Checkbox("Distance Filter", &app_state::get().bool_distance_filter)) {
				app_state::get().invalidate_ui = true;
				printf(" Distance filter \n");
			};

			if (ImGui::Checkbox("Ground plane", &app_state::get().show_ground_plane)) {
				app_state::get().invalidate_ui = true;
				printf(" Show ground plane \n");
			};

			if (ImGui::Checkbox("Ground alignment", &app_state::get().ground_alignment)) {
				app_state::get().invalidate_ui = true;
				printf(" Align ground \n");
			};

			/*
			if (ImGui::Checkbox("Ground alignment X", &app_state::get().ground_alignment_x)) {
				app_state::get().invalidate_ui = true;
				printf(" Align ground X \n");
			};

			if (ImGui::Checkbox("Ground alignment Y", &app_state::get().ground_alignment_y)) {
				app_state::get().invalidate_ui = true;
				printf(" Align ground Y \n");
			};
			*/

			if (ImGui::Checkbox("Show floor", &app_state::get().show_floor)) {
				app_state::get().invalidate_ui = true;
				printf(" Show floor \n");
			};

			ImGui::Separator();
			ImGui::Text("-- Cloud --");
			//ImGui::Text("Raw %d", cloud_raw->points.size());
			//ImGui::Text("Process %d", cloud->points.size());

			ImGui::End();
		}

	};
}