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
// [SECTION]  App: Debug Console / ShowAppConsole()
//-----------------------------------------------------------------------------

// Demonstrate creating a simple console window, with scrolling, filtering, completion and history.
// For the console , here we are using a more C++ like approach of declaring a class to hold the data and the functions.
struct AppConsole
{
	char                  InputBuf[256];
	ImVector<char*>       Items;
	bool                  ScrollToBottom;
	ImVector<char*>       History;
	int                   HistoryPos;    // -1: new line, 0..History.Size-1 browsing history.
	ImVector<const char*> Commands;

	AppConsole()
	{
		ClearLog();
		memset(InputBuf, 0, sizeof(InputBuf));
		HistoryPos = -1;
		Commands.push_back("HELP");
		Commands.push_back("HISTORY");
		Commands.push_back("CLEAR");
		Commands.push_back("CLASSIFY");  // "classify" is only here to provide an  of "C"+[tab] completing to "CL" and displaying matches.
		AddLog("Welcome to Earth Rover processing");
	}
	~AppConsole()
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

		//ImGui::TextWrapped("This  implements a console with basic coloring, completion and history. A more elaborate implementation may want to store entries along with extra data such as timestamp, emitter, etc.");
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
		// and appending newly elements as they are inserted. This is left as a task to the user until we can manage to improve this  code!
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
		AppConsole* console = (AppConsole*) data->UserData;
		return console->TextEditCallback(data);
	}

	int     TextEditCallback(ImGuiInputTextCallbackData* data)
	{
		//AddLog("cursor: %d, selection: %d-%d", data->CursorPos, data->SelectionStart, data->SelectionEnd);
		switch (data->EventFlag) {
			case ImGuiInputTextFlags_CallbackCompletion:
			{
				//  of TEXT COMPLETION

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
				//  of HISTORY
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

void ShowAppConsole(bool* p_open)
{
	if (!*p_open)
		return;

	static AppConsole console;
	console.Draw("Console", p_open);
}

//-----------------------------------------------------------------------------
// [SECTION]  App: Debug Log / ShowAppLog()
//-----------------------------------------------------------------------------

// Usage:
//  static AppLog my_log;
//  my_log.AddLog("Hello %d world\n", 123);
//  my_log.Draw("title");
struct AppLog
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
void ShowAppLog(bool* p_open)
{
	static AppLog log;

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

void ShowAppCustomRendering(bool* p_open)
{
	ImGui::SetNextWindowSize(ImVec2(350, 560), ImGuiCond_FirstUseEver);
	if (!ImGui::Begin("Plant Evaluation", p_open)) {
		ImGui::End();
		return;
	}

	// Tip: If you do a lot of custom rendering, you probably want to use your own geometrical types and benefit of overloaded operators, etc.
	// Define IM_VEC2_CLASS_EXTRA in imconfig.h to create implicit conversions between your types and ImVec2/ImVec4.
	// ImGui defines overloaded operators but they are internal to imgui.cpp and not exposed outside (to avoid messing with your types)
	// In this example we are not using the maths operators!
	ImDrawList* draw_list = ImGui::GetWindowDrawList();

	// Primitives
	ImGui::Text("Primitives");
	static float sz = 36.0f;
	static float thickness = 4.0f;
	static ImVec4 col = ImVec4(1.0f, 1.0f, 0.4f, 1.0f);
	const ImU32 col32 = ImColor(col);
	{
		const ImVec2 p = ImGui::GetCursorScreenPos();
		float x = p.x + 4.0f, y = p.y + 4.0f, spacing = 8.0f;
		for (int n = 0; n < 2; n++) {
			float curr_thickness = (n == 0) ? 1.0f : thickness;
			draw_list->AddCircle(ImVec2(x + sz*0.5f, y + sz*0.5f), sz*0.5f, col32, 20, curr_thickness); x += sz + spacing;
			draw_list->AddRect(ImVec2(x, y), ImVec2(x + sz, y + sz), col32, 0.0f, ImDrawCornerFlags_All, curr_thickness); x += sz + spacing;
			draw_list->AddRect(ImVec2(x, y), ImVec2(x + sz, y + sz), col32, 10.0f, ImDrawCornerFlags_All, curr_thickness); x += sz + spacing;
			draw_list->AddRect(ImVec2(x, y), ImVec2(x + sz, y + sz), col32, 10.0f, ImDrawCornerFlags_TopLeft | ImDrawCornerFlags_BotRight, curr_thickness); x += sz + spacing;
			draw_list->AddTriangle(ImVec2(x + sz*0.5f, y), ImVec2(x + sz, y + sz - 0.5f), ImVec2(x, y + sz - 0.5f), col32, curr_thickness); x += sz + spacing;
			draw_list->AddLine(ImVec2(x, y), ImVec2(x + sz, y), col32, curr_thickness); x += sz + spacing;   // Horizontal line (note: drawing a filled rectangle will be faster!)
			draw_list->AddLine(ImVec2(x, y), ImVec2(x, y + sz), col32, curr_thickness); x += spacing;      // Vertical line (note: drawing a filled rectangle will be faster!)
			draw_list->AddLine(ImVec2(x, y), ImVec2(x + sz, y + sz), col32, curr_thickness); x += sz + spacing;   // Diagonal line
			draw_list->AddBezierCurve(ImVec2(x, y), ImVec2(x + sz*1.3f, y + sz*0.3f), ImVec2(x + sz - sz*1.3f, y + sz - sz*0.3f), ImVec2(x + sz, y + sz), col32, curr_thickness);
			x = p.x + 4;
			y += sz + spacing;
		}
		draw_list->AddCircleFilled(ImVec2(x + sz*0.5f, y + sz*0.5f), sz*0.5f, col32, 32); x += sz + spacing;
		draw_list->AddRectFilled(ImVec2(x, y), ImVec2(x + sz, y + sz), col32); x += sz + spacing;
		draw_list->AddRectFilled(ImVec2(x, y), ImVec2(x + sz, y + sz), col32, 10.0f); x += sz + spacing;
		draw_list->AddRectFilled(ImVec2(x, y), ImVec2(x + sz, y + sz), col32, 10.0f, ImDrawCornerFlags_TopLeft | ImDrawCornerFlags_BotRight); x += sz + spacing;
		draw_list->AddTriangleFilled(ImVec2(x + sz*0.5f, y), ImVec2(x + sz, y + sz - 0.5f), ImVec2(x, y + sz - 0.5f), col32); x += sz + spacing;
		draw_list->AddRectFilled(ImVec2(x, y), ImVec2(x + sz, y + thickness), col32); x += sz + spacing;          // Horizontal line (faster than AddLine, but only handle integer thickness)
		draw_list->AddRectFilled(ImVec2(x, y), ImVec2(x + thickness, y + sz), col32); x += spacing + spacing;     // Vertical line (faster than AddLine, but only handle integer thickness)
		draw_list->AddRectFilled(ImVec2(x, y), ImVec2(x + 1, y + 1), col32);          x += sz;                  // Pixel (faster than AddLine)
		draw_list->AddRectFilledMultiColor(ImVec2(x, y), ImVec2(x + sz, y + sz), IM_COL32(0, 0, 0, 255), IM_COL32(255, 0, 0, 255), IM_COL32(255, 255, 0, 255), IM_COL32(0, 255, 0, 255));
		ImGui::Dummy(ImVec2((sz + spacing) * 8, (sz + spacing) * 3));
	}
	ImGui::Separator();
	{
		static ImVector<ImVec2> points;
		static bool adding_line = false;
		ImGui::Text("Canvas example");
		if (ImGui::Button("Clear")) points.clear();

		// Here we are using InvisibleButton() as a convenience to 1) advance the cursor and 2) allows us to use IsItemHovered()
		// But you can also draw directly and poll mouse/keyboard by yourself. You can manipulate the cursor using GetCursorPos() and SetCursorPos().
		// If you only use the ImDrawList API, you can notify the owner window of its extends by using SetCursorPos(max).
		ImVec2 canvas_pos = ImGui::GetCursorScreenPos();            // ImDrawList API uses screen coordinates!
		ImVec2 canvas_size = ImGui::GetContentRegionAvail();        // Resize canvas to what's available
		if (canvas_size.x < 50.0f) canvas_size.x = 50.0f;
		if (canvas_size.y < 50.0f) canvas_size.y = 50.0f;
		draw_list->AddRectFilledMultiColor(canvas_pos, ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y), IM_COL32(50, 50, 50, 255), IM_COL32(50, 50, 60, 255), IM_COL32(60, 60, 70, 255), IM_COL32(50, 50, 60, 255));
		draw_list->AddRect(canvas_pos, ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y), IM_COL32(255, 255, 255, 255));

		bool adding_preview = false;
		ImGui::InvisibleButton("canvas", canvas_size);
		ImVec2 mouse_pos_in_canvas = ImVec2(ImGui::GetIO().MousePos.x - canvas_pos.x, ImGui::GetIO().MousePos.y - canvas_pos.y);
		if (adding_line) {
			adding_preview = true;
			points.push_back(mouse_pos_in_canvas);
			if (!ImGui::IsMouseDown(0))
				adding_line = adding_preview = false;
		}
		if (ImGui::IsItemHovered()) {
			if (!adding_line && ImGui::IsMouseClicked(0)) {
				points.push_back(mouse_pos_in_canvas);
				adding_line = true;
			}
			if (ImGui::IsMouseClicked(1) && !points.empty()) {
				adding_line = adding_preview = false;
				points.pop_back();
				points.pop_back();
			}
		}

		draw_list->PushClipRect(canvas_pos, ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y), true);      // clip lines within the canvas (if we resize it, etc.)
		for (int i = 0; i < points.Size - 1; i += 2)
			draw_list->AddLine(ImVec2(canvas_pos.x + points[i].x, canvas_pos.y + points[i].y), ImVec2(canvas_pos.x + points[i + 1].x, canvas_pos.y + points[i + 1].y), IM_COL32(255, 255, 0, 255), 2.0f);

		draw_list->AddCircle(ImVec2(canvas_pos.x + sz*0.5f, canvas_pos.y + sz*0.5f), sz*0.5f, col32, 20, thickness);

		draw_list->PopClipRect();
		if (adding_preview)
			points.pop_back();
	}
	ImGui::End();
}