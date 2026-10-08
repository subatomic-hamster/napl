#include "viewer/theme.h"

#include <imgui_internal.h>

#include <cstddef>

extern unsigned char kUiFont[], kUiMono[];  // generated (CMakeLists.txt); non-const because ImGui takes void*, never written
extern const std::size_t kUiFontSize, kUiMonoSize;

namespace theme {
namespace {
ImFont* g_mono = nullptr;
}
ImU32 u32(ImVec4 c, float alpha) { c.w *= alpha; return ImGui::ColorConvertFloat4ToU32(c); }
ImFont* mono() { return g_mono; }

void init() {
  ImGuiIO& io = ImGui::GetIO();
  ImFontConfig cfg;
  cfg.FontDataOwnedByAtlas = false;  // the arrays are static
  io.Fonts->AddFontFromMemoryTTF(kUiFont, static_cast<int>(kUiFontSize), 15.f, &cfg);
  g_mono = io.Fonts->AddFontFromMemoryTTF(kUiMono, static_cast<int>(kUiMonoSize), 14.f, &cfg);

  ImGuiStyle& s = ImGui::GetStyle();
  s.WindowRounding = 0;
  s.ChildRounding = 2;
  s.FrameRounding = 2;
  s.PopupRounding = 3;
  s.GrabRounding = 2;
  s.ScrollbarRounding = 3;
  s.TabRounding = 0;
  s.WindowBorderSize = 0;
  s.ChildBorderSize = 0;
  s.FrameBorderSize = 0;
  s.PopupBorderSize = 1;
  s.WindowPadding = {12, 10};
  s.FramePadding = {8, 4};
  s.ItemSpacing = {8, 6};
  s.ItemInnerSpacing = {6, 4};
  s.ScrollbarSize = 8;
  s.GrabMinSize = 10;
  s.SeparatorTextBorderSize = 1;
  ImVec4* c = s.Colors;
  c[ImGuiCol_Text] = kText;
  c[ImGuiCol_TextDisabled] = kDim;
  c[ImGuiCol_WindowBg] = kPanel;
  c[ImGuiCol_ChildBg] = {0, 0, 0, 0};
  c[ImGuiCol_PopupBg] = {0.06f, 0.06f, 0.068f, 0.98f};
  c[ImGuiCol_Border] = kLine;
  c[ImGuiCol_FrameBg] = kRaised;
  c[ImGuiCol_FrameBgHovered] = {0.13f, 0.13f, 0.145f, 1.f};
  c[ImGuiCol_FrameBgActive] = {0.16f, 0.16f, 0.18f, 1.f};
  c[ImGuiCol_TitleBg] = kPanel;
  c[ImGuiCol_TitleBgActive] = kPanel;
  c[ImGuiCol_MenuBarBg] = kPanel;
  c[ImGuiCol_ScrollbarBg] = {0, 0, 0, 0};
  c[ImGuiCol_ScrollbarGrab] = {1, 1, 1, 0.18f};
  c[ImGuiCol_ScrollbarGrabHovered] = {1, 1, 1, 0.30f};
  c[ImGuiCol_ScrollbarGrabActive] = {1, 1, 1, 0.40f};
  c[ImGuiCol_CheckMark] = kAccent;
  c[ImGuiCol_SliderGrab] = kAccent;
  c[ImGuiCol_SliderGrabActive] = {0.45f, 0.86f, 1.f, 1.f};
  c[ImGuiCol_Button] = {1, 1, 1, 0.07f};
  c[ImGuiCol_ButtonHovered] = {1, 1, 1, 0.14f};
  c[ImGuiCol_ButtonActive] = {1, 1, 1, 0.22f};
  c[ImGuiCol_Header] = {1, 1, 1, 0.07f};
  c[ImGuiCol_HeaderHovered] = {1, 1, 1, 0.12f};
  c[ImGuiCol_HeaderActive] = {kAccent.x, kAccent.y, kAccent.z, 0.30f};
  c[ImGuiCol_Separator] = kLine;
  c[ImGuiCol_SeparatorHovered] = kAccent;
  c[ImGuiCol_SeparatorActive] = kAccent;
  c[ImGuiCol_ResizeGrip] = {0, 0, 0, 0};
  c[ImGuiCol_Tab] = {0, 0, 0, 0};
  c[ImGuiCol_TabHovered] = {1, 1, 1, 0.10f};
  c[ImGuiCol_TabSelected] = {1, 1, 1, 0.06f};
  c[ImGuiCol_TableHeaderBg] = {0.07f, 0.07f, 0.078f, 1.f};
  c[ImGuiCol_TableBorderStrong] = kLine;
  c[ImGuiCol_TableBorderLight] = {1, 1, 1, 0.05f};
  c[ImGuiCol_TableRowBg] = {0, 0, 0, 0};
  c[ImGuiCol_TableRowBgAlt] = {1, 1, 1, 0.025f};
  c[ImGuiCol_TextSelectedBg] = {kAccent.x, kAccent.y, kAccent.z, 0.35f};
  c[ImGuiCol_DragDropTarget] = kAccent;
  c[ImGuiCol_NavHighlight] = {0, 0, 0, 0};
}

void caption(const char* text) {
  ImGui::PushStyleColor(ImGuiCol_Text, kDim);
  ImGui::PushFont(nullptr, 11.5f);
  ImGui::TextUnformatted(text);
  ImGui::PopFont();
  ImGui::PopStyleColor();
}

int tabs(const char* const* labels, int n, int active, float gap) {
  ImDrawList* dl = ImGui::GetWindowDrawList();
  int clicked = -1;
  ImVec2 p = ImGui::GetCursorScreenPos();
  ImGui::PushFont(nullptr, 13.f);
  const float h = ImGui::GetFontSize() + 12;
  for (int i = 0; i < n; ++i) {
    const ImVec2 ts = ImGui::CalcTextSize(labels[i]);
    ImGui::SetCursorScreenPos(p);
    ImGui::PushID(i);
    if (ImGui::InvisibleButton("##tab", {ts.x + 4, h})) clicked = i;
    const bool hov = ImGui::IsItemHovered();
    ImGui::PopID();
    const ImVec4 col = i == active ? kText : hov ? ImVec4{0.85f, 0.85f, 0.87f, 1} : kDim;
    dl->AddText({p.x + 2, p.y + 6}, u32(col), labels[i]);
    if (i == active) dl->AddRectFilled({p.x, p.y + h - 2}, {p.x + ts.x + 4, p.y + h}, u32(kAccent));
    p.x += ts.x + 4 + gap;
  }
  ImGui::PopFont();
  ImGui::SetCursorScreenPos({p.x, p.y});
  return clicked;
}

float pill(ImDrawList* dl, ImVec2 pos, const char* text, ImVec4 col) {
  ImGui::PushFont(nullptr, 12.f);
  const ImVec2 ts = ImGui::CalcTextSize(text);
  dl->AddRectFilled(pos, {pos.x + ts.x + 14, pos.y + ts.y + 6}, u32({0, 0, 0, 0.55f}), 3);
  dl->AddRect(pos, {pos.x + ts.x + 14, pos.y + ts.y + 6}, u32(col, 0.55f), 3);
  dl->AddText({pos.x + 7, pos.y + 3}, u32(col), text);
  ImGui::PopFont();
  return ts.x + 14;
}

bool button(const char* label, bool accent, ImVec2 size) {
  if (accent) {
    ImGui::PushStyleColor(ImGuiCol_Button, kAccent);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4{0.45f, 0.86f, 1.f, 1.f});
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4{0.20f, 0.65f, 0.85f, 1.f});
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4{0.02f, 0.03f, 0.04f, 1.f});
  }
  const bool r = ImGui::Button(label, size);
  if (accent) ImGui::PopStyleColor(4);
  return r;
}

}  // namespace theme
