# Street Name Fixer

This utility creates a table of inconsitently named street segments as discussed
in [this issue](https://github.com/osmlab/to-fix/issues/30).

Checks for every highway if it is enclosed by two highways that share the same ref or name value.

This will typically match small segments at junctions or added motorway segments that where not tagged properly.

# Build requirements

* libosmium
* OSMPBF

# Compiling

	mkdir build
	cd build
	cmake .. -DCMAKE_BUILD_TYPE=Release
	make -j
	./streetname-fixer input.osm.pbf

This will create a file "missing_name_ref_input.osm.csv" with the following columns:

```incomplete_way_id```: OSM id of the way that has incomplete tags

```src_before_way_id```: way that conncets to the first node of the incomplete way

```src_after_way_id```: way that connects to the last node of the incomplete way

The ```src_*_way_id``` can be used to show context to the mapper, the below tags are infered from their name + ref tags.

```name```: suggestion for name tag

```ref```: suggestion for ref tag

