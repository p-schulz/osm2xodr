# osm2xodr

## Summary

The converter implements a deterministic, geometry-aware translation pipeline that transforms OpenStreetMap data into an approximate OpenDRIVE road-network representation. The workflow begins with the ingestion and parsing of OSM primitives, including roads, nodes, intersections, and traffic-control elements, followed by projection into a local metric coordinate system. Subsequent stages infer lane structure, lane widths, offsets, markings, and traffic semantics from heterogeneous OSM tags, while also resolving topological irregularities through road segmentation, merging, and junction clustering. The system then constructs road geometries, lane-level continuity relations, and intersection connectors, ensuring that road connectivity and lane assignment remain consistent across network boundaries. Finally, the inferred model is serialized as OpenDRIVE XML and accompanied by a validation report that documents inferred or lossy information. In essence, the converter performs a conservative semantic reconstruction: it prioritizes connectivity, interpretability, and practical usability over exact preservation of all original OSM semantics.

## Conversion workflow as a structured inference pipeline

The converter can be viewed as a conservative, geometry-aware reconstruction pipeline that translates OSM road topology into an approximate OpenDRIVE network. Rather than attempting a literal one-to-one transcription of OSM semantics, it performs a sequence of inference, normalization, and validation steps designed to preserve network connectivity and lane-level usability while remaining faithful to the information actually present in the source data.

### Mathematical characterization

Let an input road entity be represented by a tuple $r = (G_r, T_r)$, where $G_r = (V_r, E_r)$ is the polyline geometry and $T_r$ is the set of OSM tags attached to that road. The converter seeks an inferred model

$$
\mathcal{M} = (\mathcal{R}, \mathcal{J}, \mathcal{C})
$$

where $\mathcal{R}$ is the set of synthesized OpenDRIVE roads, $\mathcal{J}$ the set of junctions, and $\mathcal{C}$ the set of lane-level connectivity relations. The core inference function is therefore a mapping

$$
\phi: (G_r, T_r) \mapsto \Lambda_r
$$

where $\Lambda_r = (n_r^+, n_r^-, \mathbf{w}_r, \delta_r, \sigma_r)$ denotes the inferred lane plan: forward and backward lane counts, lane widths, lane offset, and traffic semantics.

In this formulation, the lane-count inference is a discrete estimator

$$
(n_r^+, n_r^-) = f_{\mathrm{lanes}}(T_r),
$$

with the base count derived from $\mathrm{lanes}$, $\mathrm{lanes:forward}$, and $\mathrm{lanes:backward}$, and with optional override by the per-lane turn-lane decoder

$$
\hat{n}_r^+ = f_{\mathrm{turn}}(T_r), \qquad \hat{n}_r^- = f_{\mathrm{turn}}^{-}(T_r).
$$

The lane-width vector is inferred as

$$
\mathbf{w}_r = \omega(T_r, \mathrm{highway}(T_r)),
$$

where $\omega$ selects explicit width values when available and otherwise falls back to a highway-class-dependent prior. The reference-line offset is then computed from the lateral balance of the left and right lane stacks:

$$
\delta_r = \frac{1}{2}\left(\sum_{i \in L_r} w_i - \sum_{j \in R_r} w_j\right),
$$

so that the reference line is centered with respect to the modeled cross-section. Traffic semantics are inferred as a classification map

$$
\tau_r : \mathrm{lane}(r) \to \mathcal{P}(\mathrm{Turn})
$$

which assigns to each lane the set of permissible movement classes such as through, left, right, slight-left, or sharp-right.

The topological normalization steps can be expressed as graph operations. Segmentation is a partition of the polyline at a set of split nodes

$$
S_r = \{v \in V_r : \deg(v) \ge 2\} \cup F,
$$

where $F$ denotes traffic-control or feature-split nodes. Merging is then a path-fusion operation over chains of fragments connected by pass-through nodes, producing a longer road segment $\bar{r}$ with an augmented lane-section sequence

$$
\bar{r} = \mathrm{Fuse}(r_1, \dots, r_k).
$$

Junction clustering is modeled as an equivalence relation over candidate junction nodes, computed by a union-find procedure over short interior links. Formally, for nodes $u$ and $v$, the relation $u \sim v$ is established when the geometric distance and interior-link conditions satisfy

$$
\mathrm{dist}(u,v) \le \tau_{\mathrm{gap}} \quad \text{and} \quad \mathrm{Interior}(u,v),
$$

which groups nearby junction nodes into a compound junction. Finally, validation is expressed as a constraint satisfaction problem over the synthesized model:

$$
\mathcal{M} \models \mathcal{V},
$$

where $\mathcal{V}$ contains continuity, compatibility, and structural constraints such as positional continuity of connector geometries, reciprocal lane links at plain road boundaries, and admissible turn-lane routing.

### 1. Inference of lane structure

Lane structure is inferred primarily from the roadway tags in infer.cpp and assembled into a lane plan object in model.hpp. The process begins with the determination of directional flow. The converter classifies a road as one-way when OSM explicitly declares `oneway=yes`, `oneway=-1`, or when the road is tagged as a roundabout or as a motorway-like facility. In the absence of such evidence, the lane count is taken to be bidirectional by default.

The lane-count inference proceeds in three stages:
- A base count is derived from `lanes`, `lanes:forward`, and `lanes:backward`.
- If the road is one-way, the forward lane count is assigned directly; otherwise separate forward and backward counts are inferred.
- If the input uses `turn:lanes` or `turn:lanes:forward/backward`, this information is treated as a more reliable per-lane source of truth than the plain `lanes=*` tag.

A particularly important refinement is the decoding of turn-lane data. The converter parses the pipe-separated slots of `turn:lanes*`, then cross-checks them against `access:lanes*` and `vehicle:lanes*` to exclude non-car lane slots such as bus or bicycle lanes that may be interleaved with ordinary traffic lanes. The resulting car-lane count can therefore override the count originally inferred from `lanes=*`. This makes the inferred lane structure robust to common OSM tagging irregularities.

The lane plan itself is represented as two physical stacks, one on each side of the road reference line. In right-hand traffic, forward-moving lanes are placed on the right side and backward-moving lanes on the left; in left-hand traffic, this convention is reversed. Sidewalks are inferred from `sidewalk`, `sidewalk:left`, and `sidewalk:right` tags and represented as additional lanes of dedicated type. A center marking is also inferred heuristically from `lane_markings`, `overtaking`, and `divider` tags.

### 2. Inference of widths and offsets

Lane widths are inferred from a hierarchy of evidence. The converter first examines explicit width declarations such as `width`, `width:lanes`, `width:lanes:forward`, and `width:lanes:backward`. When no explicit width data exists, it falls back to a highway-class-dependent default width, such as 3.75 m for motorways and 3.2 m for residential roads. If a total road width is available, the converter distributes that width across the modeled driving lanes, ensuring that the resulting lane width remains physically plausible.

A subtle but important detail is that lane widths are not always assigned positionally in a simple contiguous sequence. When `turn:lanes*` indicates that non-car slots are interleaved with car lanes, the converter preserves the original positional alignment of the car-lane slots. This prevents width assignment from being shifted by omitted slots and preserves the intended lane geometry.

The reference-line offset is then computed from the cumulative lateral extent of the left and right lane stacks. In OpenDRIVE, the reference line is not the edge of a lane but the center of the modeled cross-section. Therefore, the lane offset is chosen so that the lane stacks balance around that reference line. In more advanced cases, such as lane split/merge tapers or junction connectors, the converter uses a nonzero `lane_offset_slope` so that the reference line remains centered as widths evolve along the road.

### 3. Inference of traffic semantics

Traffic semantics are inferred at two levels: road-level direction and lane-level movement permissions. The former is straightforward and uses the usual OSM directional indicators. The latter is more sophisticated. The converter uses `turn:lanes` data to infer, for each lane, the movement directions that it is permitted to serve, such as `left`, `right`, `through`, `slight_left`, `sharp_right`, and related variants.

This information is not only used for descriptive purposes; it is also used to restrict junction routing. During connector generation, each incoming lane is matched only to outgoing lanes whose geometric movement is compatible with the lane’s permitted turn set. This makes the routing behavior materially more faithful than a purely positional matching approach. The converter also classifies movement directions from a chord-based look-ahead along the road geometry rather than from the immediate tangent of the connector, which avoids misclassifying curving ramps or slip lanes as “through” simply because their first few meters are nearly straight.

### 4. Segmentation, merging, and junction clustering

Segmentation is the first topological normalization step. In model_builder.cpp, the raw OSM way is split at every node that appears in more than one road or at special feature nodes such as traffic lights, stops, and give-way signs. These splits prevent a single OSM way from being represented as an undifferentiated polyline across multiple road junctions and preserve the topology at road boundaries.

Road merging is then applied to chains of road fragments connected by plain pass-through nodes. Such nodes are neither junctions nor traffic-control features, and the converter fuses the chain into a single, longer OpenDRIVE road. This reduces artificial fragmentation caused by OSM digitization practices. When lane structure changes across the merged chain, the converter inserts additional lane sections and tapering geometry rather than simply switching to a new road. This is one of the most important design choices in the system: it preserves continuity while avoiding abrupt lane-count discontinuities.

Junction clustering addresses a different source of fragmentation: geometrically close but distinct OSM junction nodes. The converter groups such nodes into compound junctions when the intervening road segments are short and do not contain traffic-control features. In practice, this collapses a cluster of nearby signaled or unsignaled intersections into a single junction object and allows connector roads to span the compound structure more naturally. The decision is made by a union-find procedure over the candidate nodes and the short interior links between them.

### 5. Validation procedure

Validation is performed at two levels. First, the converter supports optional read-back validation with libOpenDRIVE through the `--validate` option. When built with validation support, the generated OpenDRIVE file is parsed back by libOpenDRIVE, which confirms that the emitted XML is structurally acceptable and can be consumed by the library.

Second, the repository includes regression scripts in test that validate the generated output independently of the converter’s own internal state. These scripts check:
- Junction continuity, ensuring that synthetic connector roads align in both position and heading with their incoming and outgoing lanes.
- Turn-lane routing, verifying that a lane restricted by `turn:lanes` is only connected to geometrically compatible movements.
- Road-link continuity, ensuring that plain road-to-road boundaries maintain reciprocal lane-level links.

These checks use explicit geometric criteria and are intentionally independent of the generator’s internal assumptions. They provide a practical validation layer for the most failure-prone aspects of the conversion process: connector geometry, lane routing, and lane continuity.

In summary, the converter’s workflow is a principled attempt to recover a usable OpenDRIVE model from imperfect and semantically incomplete OSM data. Its central strategy is to infer the most defensible lane and topological structure possible, then validate the result against both formal XML-level requirements and geometry-based continuity constraints.


### Formal Defintion