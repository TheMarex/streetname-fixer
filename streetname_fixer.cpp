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

#include <unordered_map>

#include <osmium/io/pbf_input.hpp>
#include <osmium/tags/filter.hpp>
#include <osmium/index/multimap/vector.hpp>

struct ParsedWay
{
    ParsedWay(osmium::object_id_type id,
              unsigned long firstNodeId,
              unsigned long lastNodeId,
              unsigned nameId,
              unsigned refID)
    : id(id)
    , firstNodeId(firstNodeId)
    , lastNodeId(lastNodeId)
    , nameId(nameId)
    , refID(refID)
    {
    }

    osmium::object_id_type id;
    osmium::object_id_type firstNodeId;
    osmium::object_id_type lastNodeId;

    unsigned nameId;
    unsigned refID;
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
        highway_filter.add(true, "highway");
        getStringID("0");
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

    osmium::tags::KeyFilter highway_filter;

    std::unique_ptr<EndpointWayMapT> endpoint_way_map;
    std::unique_ptr<ParsedWayVectorT> parsed_ways;
    std::unique_ptr<StringTableT> string_table;

    std::unordered_map<std::string, unsigned> string_id_map;
};


struct MissingNameError
{
    MissingNameError(osmium::object_id_type startWay,
                     osmium::object_id_type enclosedWay,
                     osmium::object_id_type endWay,
                     const char* name)
    : startWay(startWay)
    , enclosedWay(enclosedWay)
    , endWay(endWay)
    , name(name)
    {
    }

    osmium::object_id_type startWay;
    osmium::object_id_type enclosedWay;
    osmium::object_id_type endWay;

    const char* name;
};

class MissingNameDetector
{
public:
    MissingNameDetector(const EndpointWayMapT& endpoint_way_map,
                        const ParsedWayVectorT& parsed_ways,
                        const StringTableT& string_table)
    : endpoint_way_map(endpoint_way_map)
    , parsed_ways(parsed_ways)
    , string_table(string_table)
    {
    }

    std::vector<MissingNameError> operator()() const
    {
        std::vector<MissingNameError> errors;
        for (const auto& way : parsed_ways)
        {
            // skip named streets
            if (way.nameId != NO_NAME_ID)
            {
                continue;
            }

            const auto beforeWayRange = endpoint_way_map.get_all(way.firstNodeId);
            const auto afterWayRange = endpoint_way_map.get_all(way.lastNodeId);

            for (auto beforeIt = beforeWayRange.first; beforeIt != beforeWayRange.second; beforeIt++)
            {
                const auto& beforeWay = parsed_ways[beforeIt->second];
                for (auto afterIt = afterWayRange.first; afterIt != afterWayRange.second; afterIt++)
                {
                    const auto& afterWay = parsed_ways[afterIt->second];

                    // Found enclosing ways with same name
                    if (beforeWay.nameId == afterWay.nameId)
                    {
                        errors.emplace_back(beforeWay.id, way.id, afterWay.id, string_table[beforeWay.nameId].c_str());
                    }
                }
            }
        }

        return errors;
    }

    const EndpointWayMapT& endpoint_way_map;
    const ParsedWayVectorT& parsed_ways;
    const StringTableT& string_table;
};

void parseInput(const char* path,
                std::unique_ptr<EndpointWayMapT>& endpoint_way_map,
                std::unique_ptr<ParsedWayVectorT>& parsed_ways,
                std::unique_ptr<StringTableT>& string_table )
{
    osmium::io::File input(path);
    osmium::io::Reader reader(input, osmium::osm_entity_bits::way);

    BufferParser parser;
    while (osmium::memory::Buffer buffer = reader.read()) {
        parser(buffer);
    }

    endpoint_way_map = std::move(parser.getEndpointWayMap());
    // after the insersion finished we need to build the index
    endpoint_way_map->consolidate();
    parsed_ways = std::move(parser.getParsedWays());
    string_table = std::move(parser.getStringTable());
}

int main (int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cout << "Error: Not enough arguments.\nUsage: ./streetname_fixer INPUT.pbf" << std::endl;
        return 1;
    }

    char* input_file_path = argv[1];
    std::unique_ptr<EndpointWayMapT> endpoint_way_map;
    std::unique_ptr<ParsedWayVectorT> parsed_ways;
    std::unique_ptr<StringTableT> string_table;
    parseInput(input_file_path, endpoint_way_map, parsed_ways, string_table);

    MissingNameDetector detector(*endpoint_way_map, *parsed_ways, *string_table);
    std::vector<MissingNameError> errors = detector();

    std::cout << "Number of errors: " << errors.size() << std::endl;

    return 0;
}
