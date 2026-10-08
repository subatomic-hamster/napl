#pragma once
// The viewer's look: near-black flat panels, white text, one accent colour, thin underline for the active tab (after the
// Waterloo sim's HUD: small black boxes, white uppercase sans text). Embedded fonts: Roboto Medium (UI) and Cousine (code / logs),
// both Apache-2.0 from Dear ImGui's misc/fonts (cmake/deps.cmake).
#include <imgui.h>

namespace theme {
constexpr ImVec4 kBg{0.035f, 0.035f, 0.040f, 1.f};      // page background (library / program pages)
constexpr ImVec4 kPanel{0.045f, 0.045f, 0.050f, 0.94f};  // side panels over the scene
constexpr ImVec4 kRaised{0.085f, 0.085f, 0.095f, 1.f};   // fields, cards
constexpr ImVec4 kLine{1.f, 1.f, 1.f, 0.10f};
constexpr ImVec4 kText{0.93f, 0.93f, 0.94f, 1.f};
constexpr ImVec4 kDim{0.58f, 0.59f, 0.62f, 1.f};
constexpr ImVec4 kAccent{0.30f, 0.78f, 0.98f, 1.f};
constexpr ImVec4 kOk{0.45f, 0.85f, 0.55f, 1.f};
constexpr ImVec4 kWarn{1.f, 0.78f, 0.30f, 1.f};
constexpr ImVec4 kErr{1.f, 0.38f, 0.38f, 1.f};

ImU32 u32(ImVec4 c, float alpha = 1.f);
// Fonts + style. Call once after ImGui::CreateContext(). The mono font is theme::mono() (nullptr before init).
void init();
ImFont* mono();
// Section label: dim, small, uppercase (the caller passes the text already in capitals).
void caption(const char* text);
// A row of text tabs on the current window's draw list starting at the cursor; the active one gets the accent underline.
// Returns the index clicked this frame or -1.
int tabs(const char* const* labels, int n, int active, float gap = 26.f);
// Small rounded count pill ("3 ERR"); returns its width.
float pill(ImDrawList* dl, ImVec2 pos, const char* text, ImVec4 col);
// A flat button in the HUD style; accent = filled accent.
bool button(const char* label, bool accent = false, ImVec2 size = {0, 0});
}  // namespace theme
