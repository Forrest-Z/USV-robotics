/* -*- mode: c++ -*- */
#ifndef CSVSTREAM_H
#define CSVSTREAM_H
/* csvstream.h
 *
 * Andrew DeOrio <awdeorio@umich.edu>
 *
 * An easy-to-use CSV file parser for C++
 * https://github.com/awdeorio/csvstream
 */

#include <cassert>
#include <exception>
#include <fstream>
#include <iostream>
#include <map>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

// A custom exception type
class csvstream_exception : public std::exception {
 public:
  const char *what() const throw() { return msg.c_str(); }
  const std::string msg;
  csvstream_exception(const std::string &msg) : msg(msg){};
};

// csvstream interface
class csvstream {
 public:
  // Constructor from filename
  csvstream(const std::string &filename, char delimiter = ',');

  // Constructor from stream
  csvstream(std::istream &is, char delimiter = ',');

  // Destructor
  ~csvstream();

  // Return true if an error flag on underlying stream is set
  explicit operator bool() const;

  // Return header processed by constructor
  std::vector<std::string> getheader() const;

  // Stream extraction operator reads one row
  csvstream &operator>>(std::map<std::string, std::string> &row);

  // Stream extraction operator reads one row, keeping column order
  csvstream &operator>>(std::vector<std::pair<std::string, std::string> > &row);

 private:
  // Filename.  Used for error messages.
  std::string filename;

  // File stream in CSV format, used when library is called with filename ctor
  std::ifstream fin;

  // Stream in CSV format
  std::istream &is;

  // Delimiter between columns
  char delimiter;

  // Line no in file.  Used for error messages
  size_t line_no;

  // Store header column names
  std::vector<std::string> header;

  // Process header, the first line of the file
  void read_header();

  // Disable copying because copying streams is bad!
  csvstream(const csvstream &);
  csvstream &operator=(const csvstream &);
};

///////////////////////////////////////////////////////////////////////////////
// Implementation

// Read and tokenize one line from a stream
static bool read_csv_line(std::istream &is, std::vector<std::string> &data,
                          char delimiter) {
  // Add entry for first token, start with empty string
  data.clear();
  data.push_back(std::string());

  // Process one character at a time
  char c = '\0';
  enum State { BEGIN, QUOTED, QUOTED_ESCAPED, UNQUOTED, UNQUOTED_ESCAPED, END };
  State state = BEGIN;
  while (is.get(c)) {
    switch (state) {
      case BEGIN:
        // We need this state transition to properly handle cases where nothing
        // is extracted.
        state = UNQUOTED;

      case UNQUOTED:
        if (c == '"') {
          // Change states when we see a double quote
          state = QUOTED;
        } else if (c == '\\') {  // note this checks for a single backslash char
          state = UNQUOTED_ESCAPED;
          data.back() += c;
        } else if (c == delimiter) {
          // If you see a delimiter, then start a new field with an empty string
          data.push_back("");
        } else if (c == '\n' || c == '\r') {
          // If you see a line ending *and it's not within a quoted token*, stop
          // parsing the line.  Works for UNIX (\n) and OSX (\r) line endings.
          // Consumes the line ending character.
          state = END;
        } else {
          // Append character to current token
          data.back() += c;
        }
        break;

      case UNQUOTED_ESCAPED:
        // If a character is escaped, add it no matter what.
        data.back() += c;
        state = UNQUOTED;
        break;

      case QUOTED:
        if (c == '"') {
          // Change states when we see a double quote
          state = UNQUOTED;
        } else if (c == '\\') {
          state = QUOTED_ESCAPED;
          data.back() += c;
        } else {
          // Append character to current token
          data.back() += c;
        }
        break;

      case QUOTED_ESCAPED:
        // If a character is escaped, add it no matter what.
        data.back() += c;
        state = QUOTED;
        break;

      case END:
        if (c == '\n') {
          // Handle second character of a Windows line ending (\r\n).  Do
          // nothing, only consume the character.
        } else {
          // If this wasn't a Windows line ending, then put character back for
          // the next call to read_csv_line()
          is.unget();
        }

        // We're done with this line, so break out of both the switch and loop.
        goto multilevel_break;  // This is a rare example where goto is OK
        break;

      default:
        assert(0);
        throw state;

    }  // switch
  }    // while

multilevel_break:
  // Clear the failbit if we extracted anything.  This is to mimic the behavior
  // of getline(), which will set the eofbit, but *not* the failbit if a partial
  // line is read.
  if (state != BEGIN) is.clear();

  // Return status is the underlying stream's status
  return static_cast<bool>(is);
}

#endif
