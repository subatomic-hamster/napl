#pragma once
// What the viewer's Design window shows and sends: a `napl design build` report (<name>.report.json next to the MJCF), and the
// JSON requests / replies of the editing backend (python/napl/design/viewer.py). Header-only; a deliberately small JSON
// reader (everything is machine-written by Python's json.dumps) so the viewer needs no JSON dependency.
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace design {

struct Json {
  enum Kind { Null, Bool, Num, Str, Arr, Obj } kind = Null;
  double num = 0;
  std::string str;
  std::vector<Json> arr;
  std::vector<std::pair<std::string, Json>> obj;

  const Json& at(const std::string& k) const {
    static const Json none;
    for (const auto& [key, v] : obj)
      if (key == k) return v;
    return none;
  }
  std::string s(const std::string& k, const std::string& dflt = "") const {
    const Json& v = at(k);
    return v.kind == Str ? v.str : dflt;
  }
  double n(const std::string& k, double dflt = 0) const {
    const Json& v = at(k);
    return v.kind == Num ? v.num : dflt;
  }
};

class Parser {
 public:
  explicit Parser(const std::string& t) : t_(t) {}
  Json parse() {
    Json v = value();
    ws();
    if (i_ != t_.size()) fail("trailing characters");
    return v;
  }

 private:
  const std::string& t_;
  std::size_t i_ = 0;
  [[noreturn]] void fail(const std::string& what) const {
    throw std::runtime_error("design report: bad JSON at byte " + std::to_string(i_) + ": " + what);
  }
  void ws() {
    while (i_ < t_.size() && (t_[i_] == ' ' || t_[i_] == '\n' || t_[i_] == '\r' || t_[i_] == '\t')) ++i_;
  }
  bool eat(const char* lit) {
    const std::string l = lit;
    if (t_.compare(i_, l.size(), l) != 0) return false;
    i_ += l.size();
    return true;
  }
  static void utf8(std::string& o, unsigned c) {
    if (c < 0x80) o += static_cast<char>(c);
    else if (c < 0x800) {
      o += static_cast<char>(0xC0 | (c >> 6));
      o += static_cast<char>(0x80 | (c & 0x3F));
    } else {
      o += static_cast<char>(0xE0 | (c >> 12));
      o += static_cast<char>(0x80 | ((c >> 6) & 0x3F));
      o += static_cast<char>(0x80 | (c & 0x3F));
    }
  }
  std::string string() {
    std::string o;
    ++i_;  // opening quote
    while (i_ < t_.size() && t_[i_] != '"') {
      char c = t_[i_++];
      if (c != '\\') {
        o += c;
        continue;
      }
      if (i_ >= t_.size()) fail("unterminated escape");
      c = t_[i_++];
      switch (c) {
        case 'n': o += '\n'; break;
        case 't': o += '\t'; break;
        case 'r': o += '\r'; break;
        case 'b': o += '\b'; break;
        case 'f': o += '\f'; break;
        case 'u': {
          if (i_ + 4 > t_.size()) fail("short \\u escape");
          utf8(o, static_cast<unsigned>(std::strtoul(t_.substr(i_, 4).c_str(), nullptr, 16)));  // BMP only (no surrogate pairs)
          i_ += 4;
          break;
        }
        default: o += c;  // " \ /
      }
    }
    if (i_ >= t_.size()) fail("unterminated string");
    ++i_;
    return o;
  }
  Json value() {
    ws();
    if (i_ >= t_.size()) fail("unexpected end");
    Json v;
    const char c = t_[i_];
    if (c == '{') {
      v.kind = Json::Obj;
      ++i_;
      ws();
      if (i_ < t_.size() && t_[i_] == '}') return ++i_, v;
      for (;;) {
        ws();
        if (i_ >= t_.size() || t_[i_] != '"') fail("object key expected");
        std::string k = string();
        ws();
        if (i_ >= t_.size() || t_[i_++] != ':') fail("':' expected");
        v.obj.emplace_back(std::move(k), value());
        ws();
        if (i_ < t_.size() && t_[i_] == ',') {
          ++i_;
          continue;
        }
        if (i_ < t_.size() && t_[i_] == '}') return ++i_, v;
        fail("',' or '}' expected");
      }
    }
    if (c == '[') {
      v.kind = Json::Arr;
      ++i_;
      ws();
      if (i_ < t_.size() && t_[i_] == ']') return ++i_, v;
      for (;;) {
        v.arr.push_back(value());
        ws();
        if (i_ < t_.size() && t_[i_] == ',') {
          ++i_;
          continue;
        }
        if (i_ < t_.size() && t_[i_] == ']') return ++i_, v;
        fail("',' or ']' expected");
      }
    }
    if (c == '"') {
      v.kind = Json::Str;
      v.str = string();
      return v;
    }
    if (eat("true") || eat("false")) {
      v.kind = Json::Bool;
      v.num = t_[i_ - 1] == 'e' && t_[i_ - 2] == 'u' ? 1 : 0;  // "true" ends in "ue", "false" in "se"
      return v;
    }
    if (eat("null")) return v;
    v.kind = Json::Num;  // numbers, plus the NaN / Infinity tokens Python's json.dumps may emit
    if (eat("NaN")) return v.num = 0, v;
    const bool neg = t_.compare(i_, 1, "-") == 0;
    if (eat("Infinity") || (neg && eat("-Infinity"))) return v.num = neg ? -1e300 : 1e300, v;
    char* end = nullptr;
    v.num = std::strtod(t_.c_str() + i_, &end);
    if (end == t_.c_str() + i_) fail("value expected");
    i_ = static_cast<std::size_t>(end - t_.c_str());
    return v;
  }
};

struct Instance {
  std::string id, part, group, body;
  double mass = 0, tol = 0;
};
struct Finding {
  std::string code, severity, message, fix;
  std::vector<std::string> where;      // instance / port / wire / mate ids
  std::vector<std::string> instances;  // the instances `where` refers to (resolved by the Python side; empty in a plain report file)
};
struct Target {
  std::string name;
  double model = 0, target = 0, gap = 0;
  bool within = false;
};
struct Report {
  std::string assembly;
  double mass_total = 0;
  std::vector<Instance> instances;
  std::vector<std::pair<std::string, double>> by_group;  // kg
  std::vector<Target> targets;
  std::vector<Finding> findings;
};

inline Report report_from(const Json& j) {
  if (j.kind != Json::Obj) throw std::runtime_error("design report: not a JSON object");
  Report r;
  r.assembly = j.s("assembly");
  r.mass_total = j.at("mass").n("total");
  for (const Json& i : j.at("instances").arr)
    r.instances.push_back({i.s("id"), i.s("part"), i.s("group"), i.s("body", i.s("id")), i.n("mass"), i.n("mass_tol")});
  for (const auto& [g, m] : j.at("mass").at("by_group").obj) r.by_group.emplace_back(g, m.num);
  for (const Json& t : j.at("targets").arr)
    r.targets.push_back({t.s("name"), t.n("model"), t.n("target"), t.n("gap"), t.at("within").kind == Json::Bool && t.at("within").num != 0});
  for (const Json& f : j.at("findings").arr) {
    Finding x{f.s("code"), f.s("severity"), f.s("message"), f.s("fix"), {}, {}};
    for (const Json& w : f.at("where").arr) x.where.push_back(w.str);
    for (const Json& w : f.at("instances").arr) x.instances.push_back(w.str);
    r.findings.push_back(std::move(x));
  }
  return r;
}

inline Report parse_report(const std::string& text) { return report_from(Parser(text).parse()); }

// ---- the editing backend (python/napl/design/viewer.py): JSON requests out, replies in -----------------------------------
inline std::string quote(const std::string& s) {
  std::string o = "\"";
  for (const char c : s) {
    if (c == '"' || c == '\\') (o += '\\') += c;
    else if (static_cast<unsigned char>(c) < 0x20) o += ' ';
    else o += c;
  }
  return o + '"';
}
// {"tool": ..., "args": {...}}; `args` is JSON text. replace_last: undo the last edit, then apply (design_edit only).
inline std::string request(const std::string& tool, const std::string& args = "{}", bool replace_last = false) {
  return "{\"tool\":" + quote(tool) + ",\"args\":" + args + (replace_last ? ",\"replace_last\":true" : "") + "}";
}

struct Port {
  std::string name, kind, type;
  bool used = false;
  bool placed = false;      // pos / axis valid (mech ports of a built design: world frame)
  double pos[3] = {0, 0, 0}, axis[3] = {0, 0, 1};
};
// World geometry of one instance as built (backend `geo`): part-frame origin + rotation, AABB, oriented boxes of its shapes.
struct GeoBox {
  double c[3], R[9], h[3];
};
struct Geo {
  double pos[3] = {0, 0, 0}, R[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}, lo[3] = {0, 0, 0}, hi[3] = {0, 0, 0};
  std::vector<GeoBox> boxes;
  bool has_color = false;
  float color[4] = {0, 0, 0, 1};
};
// What the backend reports about the design as built: the report (mass, instances, DRC findings) plus the model file and ports.
struct State {
  std::string rev, xml, build_error, path;  // xml empty: the design does not build (build_error says why)
  int undo = 0, redo = 0;
  Report report;
  std::map<std::string, std::vector<Port>> ports;  // instance id -> its ports
  std::map<std::string, Geo> geo;
  std::string root;               // id of the root instance
  std::vector<std::string> free;  // instances placed freely (pos / rpy in the document) rather than held by a mate
};
inline void vec_from(const Json& a, double* out, int n) {
  for (int i = 0; i < n && i < static_cast<int>(a.arr.size()); ++i) out[i] = a.arr[i].num;
}
inline State state_from(const Json& j) {
  State s;
  s.rev = j.s("rev");
  s.xml = j.s("xml");
  s.build_error = j.s("build_error");
  s.path = j.s("path");
  s.undo = static_cast<int>(j.n("undo"));
  s.redo = static_cast<int>(j.n("redo"));
  s.report = report_from(j.at("report"));
  for (const auto& [id, list] : j.at("ports").obj) {
    std::vector<Port>& v = s.ports[id];
    for (const Json& p : list.arr) {
      Port q{p.s("name"), p.s("kind"), p.s("type"), p.at("used").num != 0};
      if (p.at("pos").kind == Json::Arr) {
        q.placed = true;
        vec_from(p.at("pos"), q.pos, 3);
        vec_from(p.at("axis"), q.axis, 3);
      }
      v.push_back(q);
    }
  }
  for (const auto& [id, g] : j.at("geo").obj) {
    Geo& o = s.geo[id];
    vec_from(g.at("pos"), o.pos, 3);
    vec_from(g.at("R"), o.R, 9);
    vec_from(g.at("lo"), o.lo, 3);
    vec_from(g.at("hi"), o.hi, 3);
    for (const Json& b : g.at("boxes").arr) {
      GeoBox x{};
      vec_from(b.at("c"), x.c, 3);
      vec_from(b.at("R"), x.R, 9);
      vec_from(b.at("h"), x.h, 3);
      o.boxes.push_back(x);
    }
    if (g.at("color").kind == Json::Arr && g.at("color").arr.size() >= 3) {
      o.has_color = true;
      for (int i = 0; i < 4; ++i) o.color[i] = i < static_cast<int>(g.at("color").arr.size()) ? static_cast<float>(g.at("color").arr[i].num) : 1.f;
    }
  }
  s.root = j.s("root");
  for (const Json& f : j.at("free").arr) s.free.push_back(f.str);
  return s;
}

inline Report load_report(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) throw std::runtime_error("design report: cannot read " + path);
  std::ostringstream ss;
  ss << in.rdbuf();
  return parse_report(ss.str());
}

// The bus prim path of body `name` (".../name"), or "" -- bodies nest, so match the last path segment.
inline std::string body_path(const std::vector<std::string>& prims, const std::string& name) {
  for (const std::string& p : prims)
    if (p.size() > name.size() && p.compare(p.size() - name.size() - 1, std::string::npos, "/" + name) == 0) return p;
  return "";
}

}  // namespace design
