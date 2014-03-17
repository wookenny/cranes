#include "./Common.h"
#include <boost/regex.hpp>
#include <boost/filesystem.hpp>
#include <vector>
#include <string>
#include <sstream>
#include <iterator>
#include <algorithm>
#include <iostream>

using std::vector;
using std::string;
using std::cerr;
using std::cout;
using std::endl;
using std::tuple;
using std::get;
using std::to_string;
using std::stringstream;
using std::istringstream;
using std::istream_iterator;

namespace fs = boost::filesystem;

vector<string> split(const string &input) {
    istringstream buffer(input);
    vector<std::string> ret{istream_iterator<string>(buffer),
                            istream_iterator<string>()};
    return ret;
}

vector<string> &split(const string &s, char delim, vector<string> &elems) {
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

vector<string> split(const string &s, char delim) {
    vector<string> elems;
    split(s, delim, elems);
    return elems;
}

vector<string> create_interval(const string& s) {
    vector<string> result;
    int lower = 1, upper = 0;
    boost::cmatch cm;

    // interval a-d -> {a,b,c,d}
    boost::regex interval_1("\\s*(-?[0-9]+)-(-?[0-9]+)");
    if (boost::regex_match(s.c_str(), cm, interval_1)) {
        lower = stoi(cm[1]);
        upper = stoi(cm[2]);
    }

    // interval [a,d] -> {a,b,c,d}
    boost::regex interval_2("\\s*\\[(-?[0-9]+),(-?[0-9]+)\\]");
    if (boost::regex_match(s.c_str(), cm, interval_2)) {
        lower = stoi(cm[1]);
        upper = stoi(cm[2]);
    }

    // interval a -> {a}
    boost::regex interval_3("\\s*\\{?(-?[0-9]+)\\}?");
    if (boost::regex_match(s.c_str(), cm, interval_3)) {
        result.push_back(cm[1]);
        return result;
    }

    // interval a,b,d -> {a,b,d}
    boost::regex interval_4("((-?[0-9]+,)*(-?[0-9]+))");
    if (boost::regex_match(s.c_str(), cm, interval_4)) {
        for (auto s : split(cm[1], ','))
            result.push_back(s);
        return result;
    }

    while (lower <= upper) {
        result.push_back(to_string(lower));
        ++lower;
    }

    return result;
}

tuple<string, string> fixed_prefix_path(const string &s);

// finds all files or directory, matching the filter, inside the given path
vector<string> find_content(const string& path, const string& filter,
                            bool directories) {
    const boost::regex regex(filter);
    vector<string> all_matching_files;

    if ( not fs::exists(path) or not fs::is_directory(path) )
        return all_matching_files;

    fs::directory_iterator end_itr;  // Default ctor yields past-the-end
    for (fs::directory_iterator i(path); i != end_itr; ++i) {
        // Skip if not a file and looking for files
        if (not directories and !fs::is_regular_file( i->status() ) ) continue;
        // Skip if not a path and looking for paths
        if (directories and !fs::is_directory( i->status() ) ) continue;

        boost::smatch what;
        // Skip if no match
        if ( !boost::regex_match( i->path().string(),
                                    what,
                                    boost::regex(filter) ) )
            continue;

        // File matches, store it
        all_matching_files.push_back(i->path().string());
    }
    return all_matching_files;
}

// recursive function used in find files
void find_files_recursive(const string& path, const string& filter,
                                vector<string>& results) {
    // look for path separators
    auto found = filter.find("|");

    // path separator in filter -> find all matching folders and recurse
    if (found != string::npos) {
        string path_filter = filter.substr(0, found);
        auto content = find_content(path, path_filter, true);
        for (auto folder : content)
            find_files_recursive(folder, filter.substr(found+1), results);

        // done with all recursive calls!
        return;
    } else {   // no path separator in filter -> look for files
        auto content = find_content(path, filter, false);
        results.insert(end(results), begin(content), end(content));
    }
}

// find files with the given path, wildcard can be applied
vector<string> find_files(const string& filter) {
    vector<string> all_matching_files;
    // find fixed prefix
    tuple<string, string> path = fixed_prefix_path(filter);
    string prefix  = get<0>(path);
    string postfix = get<1>(path);

    // adjust filter:
    string my_filter = postfix;
    replaceAll(my_filter, "/", "|");
    replaceAll(my_filter, ".", "\\.");
    replaceAll(my_filter, "*", ".*");
    // append ./ if nothing similar in front
    find_files_recursive(prefix, my_filter, all_matching_files);
    return all_matching_files;
}

// replaces all strings from in str with to
void replaceAll(std::string& str, const std::string& from,
                                  const std::string& to) {
    if (from.empty())
        return;
    size_t start_pos = 0;
    while ((start_pos = str.find(from, start_pos)) != std::string::npos) {
        str.replace(start_pos, from.length(), to);
        // In case 'to' contains 'from', like replacing 'x' with 'yx'
        start_pos += to.length();
    }
}

tuple<string, string> fixed_prefix_path(const std::string &s) {
    auto vec = split(s, '/');
    string prefix;
    string postfix;
    auto wildcard = [](const string& s) {
                        return s.find("*") != string::npos;};

    bool prefix_end = false;
    // first
    assert(vec.size() > 0);

    for (uint i = 0; i < vec.size(); ++i) {
        if (prefix_end) {
            postfix+=vec[i]+"/";
            continue;
        }
        if ( not wildcard(vec[i]) ) {
            prefix+= vec[i]+"/";
        } else {
            postfix+=vec[i]+"/";
            prefix_end = true;
        }
    }

    if (postfix.size() > 0 and postfix[postfix.size()-1] == '/')
        postfix = postfix.substr(0, postfix.size()-1);
    return make_tuple(prefix, postfix);
}
