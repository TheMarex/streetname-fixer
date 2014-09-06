/*

Copyright (c) 2014, Patrick Niklaus
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>
#include <unordered_map>
#include <fstream>

#include <osmium/io/pbf_input.hpp>
#include <osmium/tags/filter.hpp>
#include <osmium/index/multimap/vector.hpp>

struct ParsedWay
{
    ParsedWay(osmium::object_id_type id,
              unsigned long first_node_id,
              unsigned long last_node_id,
              unsigned name_id,
              unsigned ref_id)
    : id(id)
    , first_node_id(first_node_id)
    , last_node_id(last_node_id)
    , name_id(name_id)
    , ref_id(ref_id)
    {
    }

    osmium::object_id_type id;
    osmium::object_id_type first_node_id;
    osmium::object_id_type last_node_id;

    unsigned name_id;
    unsigned ref_id;
};

using EndpointWayMapT = osmium::index::multimap::VectorBasedSparseMultimap<unsigned long, unsigned, std::vector>;
using StringTableT = std::vector<std::string>;
using ParsedWayVectorT = std::vector<ParsedWay>;

constexpr unsigned NO_NAME_ID = 0;

class BufferParser
{
public:
    BufferParser()
    : endpoint_way_map(new EndpointWayMapT())
    , parsed_ways(new ParsedWayVectorT())
    , string_table(new StringTableT())
    {
        highway_filter.add(true, "highway", "trunk");
        highway_filter.add(true, "highway", "motorway");
        highway_filter.add(true, "highway", "primary");
        highway_filter.add(true, "highway", "secondary");
        highway_filter.add(true, "highway", "tertiary");
        highway_filter.add(true, "highway", "trunk_link");
        highway_filter.add(true, "highway", "motorway_link");
        highway_filter.add(true, "highway", "primary_link");
        highway_filter.add(true, "highway", "secondary_link");
        highway_filter.add(true, "highway", "tertiary_link");
        highway_filter.add(true, "highway", "residential");
        highway_filter.add(true, "highway", "service");
        getStringID("");
    }

    void operator()(const osmium::memory::Buffer& buffer)
    {
        for (const auto& item : buffer)
        {
            if (item.type() == osmium::item_type::way)
            {
                parseWay(static_cast<const osmium::Way&>(item));
            }
        }
    }

    std::unique_ptr<EndpointWayMapT> getEndpointWayMap()
    {
        return std::move(endpoint_way_map);
    }

    std::unique_ptr<ParsedWayVectorT> getParsedWays()
    {
        return std::move(parsed_ways);
    }

    std::unique_ptr<StringTableT> getStringTable()
    {
        return std::move(string_table);
    }

private:
    inline void parseWay(const osmium::Way& way)
    {
        const auto& tags = way.tags();
        auto it = std::find_if(tags.begin(), tags.end(), highway_filter);
        if (it == tags.end())
        {
            return;
        }

        const char* name = tags.get_value_by_key("name", "");
        const char* ref = tags.get_value_by_key("ref", "");

        unsigned name_id = getStringID(name);
        unsigned ref_id = getStringID(ref);

        const osmium::NodeRef& first = way.nodes().front();
        const osmium::NodeRef& last = way.nodes().back();

        // we can't use osmium ids because MultiMap expects unsigned keys
        unsigned long firstID = static_cast<unsigned long>(first.ref());
        unsigned long lastID = static_cast<unsigned long>(last.ref());

        const unsigned wayIdx = parsed_ways->size();
        endpoint_way_map->unsorted_set(firstID, wayIdx);
        endpoint_way_map->unsorted_set(lastID,  wayIdx);

        parsed_ways->emplace_back(way.id(), firstID, lastID, name_id, ref_id);
    }

    inline unsigned getStringID(const char* str)
    {
        auto it = string_id_map.find(str);
        if (it == string_id_map.end())
        {
            string_id_map[str] = string_table->size();
            string_table->push_back(str);
            return string_table->size() - 1;
        }

        return it->second;
    }

    osmium::tags::KeyValueFilter highway_filter;

    std::unique_ptr<EndpointWayMapT> endpoint_way_map;
    std::unique_ptr<ParsedWayVectorT> parsed_ways;
    std::unique_ptr<StringTableT> string_table;

    std::unordered_map<std::string, unsigned> string_id_map;
};


struct MissingNameError
{
    MissingNameError(osmium::object_id_type src_before_way,
                     osmium::object_id_type incomplete_way,
                     osmium::object_id_type src_after_way,
                     unsigned name_id,
                     unsigned ref_id)
    : src_before_way(src_before_way)
    , incomplete_way(incomplete_way)
    , src_after_way(src_after_way)
    , name_id(name_id)
    , ref_id(ref_id)
    {
    }

    osmium::object_id_type src_before_way;
    osmium::object_id_type incomplete_way;
    osmium::object_id_type src_after_way;

    unsigned name_id;
    unsigned ref_id;
};

class MissingNameDetector
{
public:
    MissingNameDetector(const EndpointWayMapT& endpoint_way_map,
                        const ParsedWayVectorT& parsed_ways)
    : endpoint_way_map(endpoint_way_map)
    , parsed_ways(parsed_ways)
    {
    }

    std::vector<MissingNameError> operator()() const
    {
        std::vector<MissingNameError> errors;
        unsigned no_names = 0;
        for (const auto& way : parsed_ways)
        {
            // skip named streets
            if (way.name_id != NO_NAME_ID && way.ref_id != NO_NAME_ID)
            {
                continue;
            }

            no_names++;

            const auto before_way_range = endpoint_way_map.get_all(way.first_node_id);
            const auto after_way_range = endpoint_way_map.get_all(way.last_node_id);

            for (auto beforeIt = before_way_range.first; beforeIt != before_way_range.second; beforeIt++)
            {
                const auto& before_way = parsed_ways[beforeIt->second];
                if (before_way.id == way.id
                || (before_way.name_id == NO_NAME_ID && before_way.ref_id == NO_NAME_ID))
                {
                    continue;
                }

                unsigned infered_name_id = way.name_id;
                unsigned infered_ref_id = way.ref_id;
                osmium::object_id_type after_way_id;
                bool infered_tag = false;

                for (auto afterIt = after_way_range.first; afterIt != after_way_range.second; afterIt++)
                {
                    const auto& after_way = parsed_ways[afterIt->second];
                    if (after_way.id == way.id
                    || (after_way.name_id == NO_NAME_ID && after_way.ref_id == NO_NAME_ID))
                    {
                        continue;
                    }

                    // Found enclosing ways with same name
                    if (infered_name_id == NO_NAME_ID
                     && before_way.name_id != NO_NAME_ID
                     && before_way.name_id == after_way.name_id)
                    {
                        infered_name_id = before_way.name_id;
                        after_way_id = after_way.id;
                        infered_tag = true;
                    }

                    // Found enclosing ways with same ref
                    if (infered_ref_id == NO_NAME_ID
                     && before_way.ref_id != NO_NAME_ID
                     && before_way.ref_id == after_way.ref_id)
                    {
                        infered_ref_id = before_way.ref_id;
                        after_way_id = after_way.id;
                        infered_tag = true;
                    }

                    // break if we could complete all tags
                    if (infered_name_id != NO_NAME_ID && infered_ref_id != NO_NAME_ID)
                    {
                        break;
                    }
                }

                if (infered_tag)
                {
                    errors.emplace_back(before_way.id,
                                        way.id,
                                        after_way_id,
                                        infered_name_id,
                                        infered_ref_id);
                }
            }
        }

        std::sort(errors.begin(), errors.end(),
                  [](const MissingNameError& e1, const MissingNameError& e2)
                  {
                    return e1.incomplete_way < e2.incomplete_way;
                  }
        );

        std::vector<MissingNameError> unique_errors;
        unique_errors.reserve(errors.size());

        // Remove errors for the same enclosed way that suggest the same name
        std::unique_copy(errors.begin(), errors.end(), std::back_inserter(unique_errors),
                  [](const MissingNameError& e1, const MissingNameError& e2)
                  {
                    return (e1.incomplete_way == e2.incomplete_way) && (e1.name_id == e2.name_id);
                  }
        );

        std::cout << "Number of ways without name: " << no_names << std::endl;

        return unique_errors;
    }
private:

    const EndpointWayMapT& endpoint_way_map;
    const ParsedWayVectorT& parsed_ways;
};

void parseInput(const char* path,
                std::unique_ptr<EndpointWayMapT>& endpoint_way_map,
                std::unique_ptr<ParsedWayVectorT>& parsed_ways,
                std::unique_ptr<StringTableT>& string_table)
{
    osmium::io::File input(path);
    osmium::io::Reader reader(input, osmium::osm_entity_bits::way);

    std::cout << "Parsing... " << std::flush;
    BufferParser parser;
    while (osmium::memory::Buffer buffer = reader.read()) {
        parser(buffer);
    }
    std::cout << " ok." << std::endl;

    endpoint_way_map = std::move(parser.getEndpointWayMap());
    // after the insersion finished we need to build the index
    endpoint_way_map->consolidate();
    parsed_ways = std::move(parser.getParsedWays());
    string_table = std::move(parser.getStringTable());

    std::cout << "Number of parsed ways: " << parsed_ways->size() << std::endl;
}

void writeOutput(const std::vector<MissingNameError>& errors, const StringTableT& string_table, const std::string& path)
{
    std::ofstream output("missing_name_ref_" + path);

    output << "incomplete_way_id,src_before_way_id,src_after_way_id,name,ref" << std::endl;
    for (const auto& e : errors)
    {
        output << e.incomplete_way << ","
               << e.src_before_way << ","
               << e.src_after_way << ","
               << string_table[e.name_id] << ","
               << string_table[e.ref_id] << std::endl;
    }
}

int main (int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cout << "Error: Not enough arguments.\nUsage: ./streetname_fixer INPUT.pbf" << std::endl;
        return 1;
    }

    char* input_file_path = argv[1];
    boost::filesystem::path input_path(input_file_path);
    if (!boost::filesystem::exists(input_path))
    {
        std::cout << "Error: Input file " << input_file_path << " does not exists." << std::endl;
    }

    boost::filesystem::path output_path = input_path.filename().replace_extension(".csv");

    std::unique_ptr<EndpointWayMapT> endpoint_way_map;
    std::unique_ptr<ParsedWayVectorT> parsed_ways;
    std::unique_ptr<StringTableT> string_table;
    parseInput(input_file_path, endpoint_way_map, parsed_ways, string_table);

    MissingNameDetector detector(*endpoint_way_map, *parsed_ways);
    std::vector<MissingNameError> errors = detector();

    writeOutput(errors, *string_table, output_path.string());

    std::cout << "Number of errors: " << errors.size() << std::endl;

    return 0;
}
