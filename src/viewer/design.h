#pragma once
// Read-only view of a `napl design build` report (<name>.report.json next to the MJCF): what the viewer's Design window
// shows. Header-only; a deliberately small JSON reader (the report is machine-written by Python's json.dumps) so the
// viewer needs no JSON dependency.
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
  std::vector<std::string> where;  // instance / port / wire / mate ids
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

inline Report parse_report(const std::string& text) {
  const Json j = Parser(text).parse();
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
    Finding x{f.s("code"), f.s("severity"), f.s("message"), f.s("fix"), {}};
    for (const Json& w : f.at("where").arr) x.where.push_back(w.str);
    r.findings.push_back(std::move(x));
  }
  return r;
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
