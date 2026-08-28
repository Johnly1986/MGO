// Copyright Johnlyon
//
// Minimal JSON value model + parser + serializer (RFC 8259 subset used by
// GeoJSON). No external dependencies; C++17; cross-platform.
//
// Design notes:
//   - Object member order is preserved (insertion order), so property
//     round-trips keep the original document shape.
//   - Numbers keep their RAW source token. Untouched numbers (properties,
//     ids, coordinates that are not transformed) round-trip byte-exact.
//   - Strings are stored as UTF-8 with escapes decoded; the serializer
//     re-escapes minimally (control chars, quote, backslash) and emits
//     non-ASCII UTF-8 verbatim.
//

#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace mgo {
namespace json {

class Value
{
public:
    enum class Type { Null, Bool, Number, String, Array, Object };

    Type type = Type::Null;
    bool boolean = false;
    std::string number;  // raw token, valid when type == Number
    std::string str;     // UTF-8, valid when type == String
    std::vector<Value> array;
    std::vector<std::pair<std::string, Value>> object;  // insertion order

    bool isNull()   const { return type == Type::Null; }
    bool isBool()   const { return type == Type::Bool; }
    bool isNumber() const { return type == Type::Number; }
    bool isString() const { return type == Type::String; }
    bool isArray()  const { return type == Type::Array; }
    bool isObject() const { return type == Type::Object; }

    // Number access (parses the raw token).
    double asDouble() const;
    // Object member lookup; nullptr when absent or not an object.
    const Value* find(const std::string& key) const;
    Value* find(const std::string& key);

    Value& set(const std::string& key, Value v);
    void pushBack(Value v);

    static Value makeNumber(double d);       // shortest round-trip token
    static Value makeString(std::string s) { Value v; v.type = Type::String; v.str = std::move(s); return v; }
};

// Parse a complete JSON document. Returns false and fills `error` on
// failure. UTF-8 BOM is tolerated. Depth is capped (GeoJSON needs < 32;
// malformed documents must not smash the stack).
bool parse(const std::string& text, Value& out, std::string& error);

// Serialize. Pretty-print with 2-space indent when `pretty`.
std::string serialize(const Value& v, bool pretty = false);

} // namespace json
} // namespace mgo
