// Reader for the golden data produced by tools/dump_golden.py.
//
// Deliberately simple format: each record starts with a "--" line, followed by
// lines of the form "key value value ...". Floats are written with float.hex()
// (%a form) and read back with std::strtod, so they are bit-exact.

#ifndef GOLDEN_READER_HPP_
#define GOLDEN_READER_HPP_

#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace golden
{

class Record
{
public:
  void add(const std::string & key, std::vector<std::string> tokens)
  {
    fields_[key] = std::move(tokens);
  }

  bool has(const std::string & key) const {return fields_.count(key) != 0;}

  const std::vector<std::string> & tokens(const std::string & key) const
  {
    const auto it = fields_.find(key);
    if (it == fields_.end()) {
      throw std::runtime_error("missing key '" + key + "' trong du lieu vang");
    }
    return it->second;
  }

  std::vector<double> doubles(const std::string & key) const
  {
    const auto & raw = tokens(key);
    std::vector<double> values;
    values.reserve(raw.size());
    for (const auto & token : raw) {
      values.push_back(std::strtod(token.c_str(), nullptr));
    }
    return values;
  }

  std::vector<int> ints(const std::string & key) const
  {
    const auto & raw = tokens(key);
    std::vector<int> values;
    values.reserve(raw.size());
    for (const auto & token : raw) {
      values.push_back(std::atoi(token.c_str()));
    }
    return values;
  }

  double scalar(const std::string & key) const
  {
    const auto values = doubles(key);
    if (values.size() != 1) {
      throw std::runtime_error("key '" + key + "' is not a single number");
    }
    return values[0];
  }

  int integer(const std::string & key) const
  {
    const auto values = ints(key);
    if (values.size() != 1) {
      throw std::runtime_error("key '" + key + "' is not a single integer");
    }
    return values[0];
  }

private:
  std::map<std::string, std::vector<std::string>> fields_;
};

inline std::vector<Record> read(const std::string & path)
{
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error(
            "cannot open golden data: " + path +
            " (run `python3 tools/dump_golden.py` first)");
  }

  std::vector<Record> records;
  Record current;
  bool started = false;
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }
    if (line == "--") {
      if (started) {
        records.push_back(current);
      }
      current = Record();
      started = true;
      continue;
    }
    std::istringstream stream(line);
    std::string key;
    stream >> key;
    std::vector<std::string> tokens;
    std::string token;
    while (stream >> token) {
      tokens.push_back(token);
    }
    current.add(key, std::move(tokens));
  }
  if (started) {
    records.push_back(current);
  }
  return records;
}

}  // namespace golden

#endif  // GOLDEN_READER_HPP_
