#include "csvstream.h"

csvstream::csvstream(const std::string &filename, char delimiter)
    : filename(filename), is(fin), delimiter(delimiter), line_no(0) {
  // Open file
  fin.open(filename.c_str());
  if (!fin.is_open()) {
    throw csvstream_exception("Error opening file: " + filename);
  }

  // Process header
  read_header();
}

csvstream::csvstream(std::istream &is, char delimiter)
    : filename("[no filename]"), is(is), delimiter(delimiter), line_no(0) {
  read_header();
}

csvstream::~csvstream() {
  if (fin.is_open()) fin.close();
}

csvstream::operator bool() const { return static_cast<bool>(is); }

std::vector<std::string> csvstream::getheader() const { return header; }

csvstream &csvstream::operator>>(std::map<std::string, std::string> &row) {
  // Clear input row
  row.clear();

  // Read one line from stream, bail out if we're at the end
  std::vector<std::string> data;
  if (!read_csv_line(is, data, delimiter)) return *this;
  line_no += 1;

  // Check length of data
  if (data.size() != header.size()) {
    auto msg = "Number of items in row does not match header. " + filename +
               ":L" + std::to_string(line_no) + " " + "header.size() = " +
               std::to_string(header.size()) + " " + "row.size() = " +
               std::to_string(data.size()) + " ";
    throw csvstream_exception(msg);
  }

  // combine data and header into a row object
  for (size_t i = 0; i < data.size(); ++i) {
    row[header[i]] = data[i];
  }

  return *this;
}

csvstream &csvstream::operator>>(
    std::vector<std::pair<std::string, std::string> > &row) {
  // Clear input row
  row.clear();
  row.resize(header.size());

  // Read one line from stream, bail out if we're at the end
  std::vector<std::string> data;
  if (!read_csv_line(is, data, delimiter)) return *this;
  line_no += 1;

  // Check length of data
  if (row.size() != header.size()) {
    auto msg = "Number of items in row does not match header. " + filename +
               ":L" + std::to_string(line_no) + " " + "header.size() = " +
               std::to_string(header.size()) + " " + "row.size() = " +
               std::to_string(row.size()) + " ";
    throw csvstream_exception(msg);
  }

  // combine data and header into a row object
  for (size_t i = 0; i < data.size(); ++i) {
    row[i] = make_pair(header[i], data[i]);
  }

  return *this;
}

void csvstream::read_header() {
  // read first line, which is the header
  if (!read_csv_line(is, header, delimiter)) {
    throw csvstream_exception("error reading header");
  }
}
