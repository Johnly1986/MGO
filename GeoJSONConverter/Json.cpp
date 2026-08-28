// Copyright Johnlyon
//
// Minimal JSON parser / serializer implementation
//

#include "Json.h"

#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdlib>
#include <cstdio>

namespace mgo {
namespace json {

namespace {

constexpr int kMaxDepth = 200;

// ---------------------------------------------------------------------------
// Parser
// ---------------------------------------------------------------------------

class Parser
{
public:
    Parser(const std::string& text) : m_s(text) {}

    bool parse(Value& out, std::string& error)
    {
        skipWs();
        if (!parseValue(out, 0))
        {
            error = m_err + " (offset " + std::to_string(m_pos) + ")";
            return false;
        }
        skipWs();
        if (m_pos != m_s.size())
        {
            error = std::string("Trailing content after document (offset ")
                    + std::to_string(m_pos) + ")";
            return false;
        }
        return true;
    }

private:
    bool fail(const char* msg)
    {
        m_err = msg;
        return false;
    }

    void skipWs()
    {
        while (m_pos < m_s.size())
        {
            char c = m_s[m_pos];
            if (c == ' ' || c == '\t' || c == '\r' || c == '\n') ++m_pos;
            else break;
        }
    }

    bool peek(char c)
    {
        skipWs();
        return m_pos < m_s.size() && m_s[m_pos] == c;
    }

    bool expect(char c)
    {
        skipWs();
        if (m_pos >= m_s.size() || m_s[m_pos] != c)
            return fail("Unexpected character");
        ++m_pos;
        return true;
    }

    bool literal(const char* lit, Value::Type t, Value& out)
    {
        size_t n = std::char_traits<char>::length(lit);
        if (m_s.compare(m_pos, n, lit) != 0) return fail("Invalid literal");
        m_pos += n;
        out.type = t;
        if (t == Value::Type::Bool) out.boolean = (lit[0] == 't');
        return true;
    }

    bool parseValue(Value& out, int depth)
    {
        if (depth > kMaxDepth) return fail("Nesting too deep");
        skipWs();
        if (m_pos >= m_s.size()) return fail("Unexpected end of input");
        char c = m_s[m_pos];
        switch (c)
        {
            case 'n': return literal("null", Value::Type::Null, out);
            case 't': return literal("true", Value::Type::Bool, out);
            case 'f': return literal("false", Value::Type::Bool, out);
            case '"':
            {
                out.type = Value::Type::String;
                return parseString(out.str);
            }
            case '[': return parseArray(out, depth);
            case '{': return parseObject(out, depth);
            default:
                if (c == '-' || (c >= '0' && c <= '9'))
                {
                    out.type = Value::Type::Number;
                    return parseNumber(out.number);
                }
                return fail("Unexpected character");
        }
    }

    bool parseNumber(std::string& token)
    {
        size_t start = m_pos;
        if (m_pos < m_s.size() && m_s[m_pos] == '-') ++m_pos;
        // int part
        if (m_pos >= m_s.size()) return fail("Invalid number");
        size_t intStart = m_pos;
        while (m_pos < m_s.size() && std::isdigit(static_cast<unsigned char>(m_s[m_pos])))
            ++m_pos;
        if (m_pos == intStart) return fail("Invalid number");
        // RFC 8259: no leading zeros ("0" alone or "0.frac" only).
        if (m_pos - intStart > 1 && m_s[intStart] == '0')
            return fail("Leading zero in number");
        // frac
        if (m_pos < m_s.size() && m_s[m_pos] == '.')
        {
            ++m_pos;
            size_t fracStart = m_pos;
            while (m_pos < m_s.size() && std::isdigit(static_cast<unsigned char>(m_s[m_pos])))
                ++m_pos;
            if (m_pos == fracStart) return fail("Invalid number");
        }
        // exponent
        if (m_pos < m_s.size() && (m_s[m_pos] == 'e' || m_s[m_pos] == 'E'))
        {
            ++m_pos;
            if (m_pos < m_s.size() && (m_s[m_pos] == '+' || m_s[m_pos] == '-')) ++m_pos;
            size_t expStart = m_pos;
            while (m_pos < m_s.size() && std::isdigit(static_cast<unsigned char>(m_s[m_pos])))
                ++m_pos;
            if (m_pos == expStart) return fail("Invalid number");
        }
        token = m_s.substr(start, m_pos - start);
        return true;
    }

    void appendUtf8(std::string& out, uint32_t cp)
    {
        if (cp < 0x80)
            out += static_cast<char>(cp);
        else if (cp < 0x800)
        {
            out += static_cast<char>(0xC0 | (cp >> 6));
            out += static_cast<char>(0x80 | (cp & 0x3F));
        }
        else if (cp < 0x10000)
        {
            out += static_cast<char>(0xE0 | (cp >> 12));
            out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
            out += static_cast<char>(0x80 | (cp & 0x3F));
        }
        else
        {
            out += static_cast<char>(0xF0 | (cp >> 18));
            out += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
            out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
            out += static_cast<char>(0x80 | (cp & 0x3F));
        }
    }

    bool parseHex4(uint32_t& v)
    {
        if (m_pos + 4 > m_s.size()) return fail("Truncated \\u escape");
        v = 0;
        for (int i = 0; i < 4; ++i)
        {
            char c = m_s[m_pos + i];
            v <<= 4;
            if (c >= '0' && c <= '9') v |= static_cast<uint32_t>(c - '0');
            else if (c >= 'a' && c <= 'f') v |= static_cast<uint32_t>(c - 'a' + 10);
            else if (c >= 'A' && c <= 'F') v |= static_cast<uint32_t>(c - 'A' + 10);
            else return fail("Invalid \\u escape");
        }
        m_pos += 4;
        return true;
    }

    bool parseString(std::string& out)
    {
        out.clear();
        if (!expect('"')) return false;
        while (true)
        {
            if (m_pos >= m_s.size()) return fail("Unterminated string");
            char c = m_s[m_pos++];
            if (c == '"') return true;
            if (static_cast<unsigned char>(c) < 0x20)
                return fail("Raw control character in string");
            if (c != '\\')
            {
                out += c;  // UTF-8 passes through verbatim
                continue;
            }
            if (m_pos >= m_s.size()) return fail("Truncated escape");
            char e = m_s[m_pos++];
            switch (e)
            {
                case '"':  out += '"';  break;
                case '\\': out += '\\'; break;
                case '/':  out += '/';  break;
                case 'b':  out += '\b'; break;
                case 'f':  out += '\f'; break;
                case 'n':  out += '\n'; break;
                case 'r':  out += '\r'; break;
                case 't':  out += '\t'; break;
                case 'u':
                {
                    uint32_t cp;
                    if (!parseHex4(cp)) return false;
                    if (cp >= 0xD800 && cp <= 0xDBFF)
                    {
                        // High surrogate: require a low surrogate.
                        if (m_pos + 1 < m_s.size() && m_s[m_pos] == '\\' && m_s[m_pos + 1] == 'u')
                        {
                            m_pos += 2;
                            uint32_t lo;
                            if (!parseHex4(lo)) return false;
                            if (lo < 0xDC00 || lo > 0xDFFF)
                                return fail("Invalid surrogate pair");
                            cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
                        }
                        else
                            return fail("Lone high surrogate");
                    }
                    else if (cp >= 0xDC00 && cp <= 0xDFFF)
                        return fail("Lone low surrogate");
                    appendUtf8(out, cp);
                    break;
                }
                default:
                    return fail("Invalid escape");
            }
        }
    }

    bool parseArray(Value& out, int depth)
    {
        out.type = Value::Type::Array;
        if (!expect('[')) return false;
        if (peek(']')) { ++m_pos; return true; }
        while (true)
        {
            Value v;
            if (!parseValue(v, depth + 1)) return false;
            out.array.push_back(std::move(v));
            skipWs();
            if (m_pos >= m_s.size()) return fail("Unterminated array");
            if (m_s[m_pos] == ',') { ++m_pos; continue; }
            if (m_s[m_pos] == ']') { ++m_pos; return true; }
            return fail("Expected ',' or ']'");
        }
    }

    bool parseObject(Value& out, int depth)
    {
        out.type = Value::Type::Object;
        if (!expect('{')) return false;
        if (peek('}')) { ++m_pos; return true; }
        while (true)
        {
            std::string key;
            skipWs();
            if (!parseString(key)) return false;
            if (!expect(':')) return false;
            Value v;
            if (!parseValue(v, depth + 1)) return false;
            out.object.emplace_back(std::move(key), std::move(v));
            skipWs();
            if (m_pos >= m_s.size()) return fail("Unterminated object");
            if (m_s[m_pos] == ',') { ++m_pos; continue; }
            if (m_s[m_pos] == '}') { ++m_pos; return true; }
            return fail("Expected ',' or '}'");
        }
    }

    std::string m_s;  // by value: Parser may outlive the caller's temporary
    size_t m_pos = 0;
    std::string m_err;
};

// ---------------------------------------------------------------------------
// Serializer
// ---------------------------------------------------------------------------

void escapeString(const std::string& in, std::string& out)
{
    out += '"';
    for (unsigned char c : in)
    {
        switch (c)
        {
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\b': out += "\\b";  break;
            case '\f': out += "\\f";  break;
            case '\n': out += "\\n";  break;
            case '\r': out += "\\r";  break;
            case '\t': out += "\\t";  break;
            default:
                if (c < 0x20)
                {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                    out += buf;
                }
                else
                    out += static_cast<char>(c);
        }
    }
    out += '"';
}

void serializeValue(const Value& v, std::string& out, bool pretty, int depth)
{
    auto newline = [&](int d) {
        if (!pretty) return;
        out += '\n';
        out.append(static_cast<size_t>(d) * 2, ' ');
    };

    switch (v.type)
    {
        case Value::Type::Null:   out += "null";  break;
        case Value::Type::Bool:   out += v.boolean ? "true" : "false"; break;
        case Value::Type::Number: out += v.number; break;
        case Value::Type::String: escapeString(v.str, out); break;
        case Value::Type::Array:
        {
            out += '[';
            if (!v.array.empty())
            {
                for (size_t i = 0; i < v.array.size(); ++i)
                {
                    if (i) out += ',';
                    newline(depth + 1);
                    serializeValue(v.array[i], out, pretty, depth + 1);
                }
                newline(depth);
            }
            out += ']';
            break;
        }
        case Value::Type::Object:
        {
            out += '{';
            if (!v.object.empty())
            {
                for (size_t i = 0; i < v.object.size(); ++i)
                {
                    if (i) out += ',';
                    newline(depth + 1);
                    escapeString(v.object[i].first, out);
                    out += pretty ? ": " : ":";
                    serializeValue(v.object[i].second, out, pretty, depth + 1);
                }
                newline(depth);
            }
            out += '}';
            break;
        }
    }
}

} // anonymous namespace

// ---------------------------------------------------------------------------

double Value::asDouble() const
{
    return std::strtod(number.c_str(), nullptr);
}

const Value* Value::find(const std::string& key) const
{
    if (type != Type::Object) return nullptr;
    for (const auto& kv : object)
        if (kv.first == key) return &kv.second;
    return nullptr;
}

Value* Value::find(const std::string& key)
{
    if (type != Type::Object) return nullptr;
    for (auto& kv : object)
        if (kv.first == key) return &kv.second;
    return nullptr;
}

Value& Value::set(const std::string& key, Value v)
{
    type = Type::Object;
    for (auto& kv : object)
        if (kv.first == key) { kv.second = std::move(v); return kv.second; }
    object.emplace_back(key, std::move(v));
    return object.back().second;
}

void Value::pushBack(Value v)
{
    type = Type::Array;
    array.push_back(std::move(v));
}

Value Value::makeNumber(double d)
{
    Value v;
    v.type = Type::Number;
    // Shortest round-trip representation (C++17 to_chars).
    char buf[64];
    auto res = std::to_chars(buf, buf + sizeof(buf), d);
    v.number.assign(buf, res.ptr);
    return v;
}

bool parse(const std::string& text, Value& out, std::string& error)
{
    // Tolerate UTF-8 BOM.
    const char* data = text.data();
    size_t size = text.size();
    if (size >= 3 && static_cast<unsigned char>(data[0]) == 0xEF
                  && static_cast<unsigned char>(data[1]) == 0xBB
                  && static_cast<unsigned char>(data[2]) == 0xBF)
    {
        data += 3;
        size -= 3;
    }
    Parser p(std::string(data, size));
    return p.parse(out, error);
}

std::string serialize(const Value& v, bool pretty)
{
    std::string out;
    out.reserve(256);
    serializeValue(v, out, pretty, 0);
    return out;
}

} // namespace json
} // namespace mgo
