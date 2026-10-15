//go:build ignore

// series188 is series186 -- near-index-matched thindielectric sheets with a
// thin-film coating painted on by a blend texture -- swept around the
// ridgerunner-tightened (7,8,9) Lissajous knot instead of 186's cable torus knot.
// The tube radius sweeps log-uniformly from .1, where the tube clears itself
// everywhere (the knot's max non-overlapping radius is ~.49), to 10, where it is
// buried many layers deep in itself, as 186's always was.
package main

import (
	"bufio"
	"encoding/binary"
	"flag"
	"fmt"
	"math"
	"os"
	"sort"
	"text/template"

	"github.com/Opioid/rgbe"
	"github.com/hunterloftis/pbr/pkg/geom"
	"github.com/rrothenb/fm-animations/utils"
)

var primes = []int{2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61, 67, 71, 73, 79, 83, 89, 97, 101, 103, 107, 109, 113, 127, 131, 137, 139, 149, 151, 157, 163, 167, 173, 179, 181, 191, 193, 197, 199, 211, 223, 227, 229, 233, 239, 241, 251, 257, 263, 269, 271}

func prime(i int) float64 {
	return float64(primes[i-1])
}

type MeshType struct {
	NumVertices int
	NumFaces    int
}

var sin = math.Sin
var cos = math.Cos
var pow = math.Pow
var sqrt = math.Sqrt
var pi = math.Pi
var abs = math.Abs
var max = math.Max

func sign(x float64) float64 {
	if x < 0 {
		return -1
	} else {
		return 1
	}
}
func spow(x, y float64) float64 {
	return sign(x) * pow(abs(x), y)
}

func strength(x float64) float64 {
	return sin(x)*.75 + 1.25
}

func circle(x float64) geom.Vec {
	return geom.Vec{sin(x), cos(x), 0}
}

// 186's camera orbit, as a direction only; the distance is set per frame from
// the size of the tangle.
func cameraDirection(t float64) geom.Vec {
	loc, _ := circle(t).Plus(geom.Vec{0, 0, .75 + 1*sin(2*t)}).Unit()
	return geom.Vec(loc)
}

func texture(a, x, y, z, t float64) float64 {
	x = x * 2 * pi
	y = y * 2 * pi
	z = z * 2 * pi
	return sin(
		x + y +
			a*strength(.1+2*t)*sin(x+a*strength(.2+3*t)*sin(x)) +
			a*strength(.3+5*t)*sin(y+a*strength(.4+7*t)*sin(y)) +
			a*strength(.5+5*t)*sin(z+a*strength(.6+7*t)*sin(z)) +
			a*strength(.7+11*t)*sin(2*x+3*y) +
			a*strength(.8+13*t)*sin(2*y-3*z) +
			a*strength(.9+17*t)*sin(3*z-5*x))
}

func shaper(x, a, b float64) float64 {
	return spow(pow(x/2+.5, pow(10, a))*2-1, pow(10, b))
}

// 186 lived in knot units where its tangle was ~4.5 across from the centre; the
// texture and blend pattern are evaluated at a point rescaled into those units,
// so their features stay the same size relative to the frame whatever the radius.
const extent186 = 4.5

// Surface displacement is tapered off within displacementCore of the centre,
// in those units.
const displacementCore = .1

// Film shape. Mitsuba's fov (fov_axis="larger") applies to the wider side,
// and is fitted per frame within these bounds.
const aspectRatio = 16.0 / 9
const minFOV, maxFOV = 10.0, 120.0

// Share of the widest frame that still fits inside the tangle's silhouette that
// is actually used: below 1 for the tangle thinning toward its edge.
const fillMargin = .85

// Percentile of the centreline's distance along each camera axis taken as the
// extent of the tangle's body.
const bodyPercentile = .8

// Sweep frequencies. At 32 frames sin(k*t) only advances k mod 32 steps per
// cycle, so most of 186's primes collapsed onto a handful of frequencies and
// whole groups of parameters moved in lockstep (film IOR was always exactly
// 1.8 + .7 * the blend exponent). Each parameter that matters now gets its own
// odd frequency -- odd, so every one of the 32 frames lands on a distinct phase --
// with sin and cos sharing a frequency where needed. The sin sweeps also carry
// their own phase: with an odd k and no phase, sin(k*t) repeats itself between
// frames f and 16-f, halving the combinations. 1, 2 and 5 are taken by the
// camera orbit and the envmap. The medium is optically negligible here (see
// series187), so its parameters get the leftover even frequencies.

func textureAmount(t float64) float64 {
	return pow(10, cos(3*t)-1)
}

func blendValue(p geom.Vec, t float64) float64 {
	return shaper(texture(1+.01*texture(textureAmount(t), p.X, p.Y, p.Z, t), p.X, p.Y, p.Z, t), blendExponent(t), 0)/2 + .5
}

// Exponent of the film/no-film blend shaper, in [-1, 0]. Kept non-positive so the
// weight leans to the uncoated face and the film shows only as sparse speckles
// and lines; positive values (mostly film) gave the loud saturated frames.
func blendExponent(t float64) float64 {
	return -.5 + .5*sin(7*t+2)
}

// Displacement amplitude as a share of the tube radius, log-uniform over
// [1%, 33%] -- deep relief at the top, but never more than a third of the way
// to the tube's centreline.
func displacementFraction(t float64) float64 {
	return pow(10, -1.24+.76*sin(19*t+6))
}

// Tube radius in knot units, log-uniform over [.36, 5]: past the ~.4 where the
// tubes stop reading as tubes, short of the ~6 where everything goes to soft blur.
func tubeRadius(t float64) float64 {
	return pow(10, .15-.6*cos(3*t))
}

func renderSurfaces(frameNumber int, dt float64, desiredTriangles int, knotPath string, width, samples int) {
	t := float64(frameNumber) * dt
	envSize := int(pow(float64(desiredTriangles), .5))

	beads, err := utils.ReadVECT(knotPath)
	if err != nil {
		panic(err)
	}
	knot := utils.FitFourier(beads, len(beads)/4)
	r := tubeRadius(t)

	// Shape and FOV are fitted to the object from wherever the camera is. In the
	// camera's frame -- image right, image up (world z with the view direction
	// projected out, as Mitsuba's lookat builds it), and the view direction --
	// the tangle is modelled as the ellipsoid its extents along those three axes
	// describe. From a camera at distance D on the view axis, an ellipsoid with
	// semi-axes a (right), b (up), c (view) subtends a/sqrt(D²-c²) horizontally
	// and b/sqrt(D²-c²) vertically in tangent space. The knot is squashed along
	// up or right to bring that silhouette to the film's 16:9, and the frame is
	// the widest 16:9 rectangle inside it -- corners on the ellipse, so the frame
	// stays filled -- less fillMargin, because the tangle is not a solid ellipsoid
	// and thins toward its edge. Depth is left alone: scaling along the view axis
	// would only change how strong the perspective is. Only the centreline is
	// reshaped; the tube is swept around it afterwards, so its cross section
	// stays round. The zoom stays where the unsquashed knot puts it.
	view := cameraDirection(t).Scaled(-1)
	up, _ := geom.Vec{0, 0, 1}.Minus(view.Scaled(view.Z)).Unit()
	imageUp := geom.Vec(up)
	right, _ := view.Cross(imageUp).Unit()
	imageRight := geom.Vec(right)

	// Dense sample of the knot, for its extents and length. The extents are the
	// body of the tangle, not its outermost points: a percentile of how far the
	// centreline gets along each axis. The extremes are reached by a strand or
	// two, and with the camera close the silhouette formula below is very
	// sensitive to the depth extent, so using them leaves the frame's corners
	// past the edge of the tangle.
	const probe = 20000
	knotSamples := make([]geom.Vec, probe)
	knotReach := 0.0
	rights, ups := make([]float64, probe), make([]float64, probe)
	for i := range knotSamples {
		p := knot.Eval(float64(i) / probe * 2 * pi)
		knotSamples[i] = p
		knotReach = max(knotReach, p.Len())
		rights[i], ups[i] = abs(p.Dot(imageRight)), abs(p.Dot(imageUp))
	}
	bodyExtent := func(values []float64) float64 {
		sort.Float64s(values)
		return values[int(bodyPercentile*float64(len(values)-1))]
	}
	alongRight, alongUp := bodyExtent(rights), bodyExtent(ups)
	// Scale the centreline so (rightScale*alongRight + r) : (upScale*alongUp + r)
	// is 16:9, squashing whichever axis is too long. A fat tube is round whatever
	// the centreline does, so the squash can't always get all the way there; the
	// frame below is fitted to whatever silhouette results.
	const leastScale = .25
	rightScale, upScale := 1.0, 1.0
	if (alongUp+r)*aspectRatio > alongRight+r {
		upScale = math.Max(leastScale, ((alongRight+r)/aspectRatio-r)/alongUp)
	} else {
		rightScale = math.Max(leastScale, ((alongUp+r)*aspectRatio-r)/alongRight)
	}
	reshaped := func(p geom.Vec) geom.Vec {
		return p.Minus(imageUp.Scaled(p.Dot(imageUp) * (1 - upScale))).Minus(imageRight.Scaled(p.Dot(imageRight) * (1 - rightScale)))
	}
	centersProbe := make([]geom.Vec, probe)
	views := make([]float64, probe)
	for i, p := range knotSamples {
		centersProbe[i] = reshaped(p)
		views[i] = abs(p.Dot(view))
	}
	alongView := bodyExtent(views)
	length := 0.0
	for i, c := range centersProbe {
		length += c.Minus(centersProbe[(i+probe-1)%probe]).Len()
	}
	// norm takes knot units to 186's knot units; world is a further .075, as in 186.
	norm := extent186 / (knotReach + r)
	world := .075 * norm
	a := (rightScale*alongRight + r) * world
	b := (upScale*alongUp + r) * world
	c := (alongView + r) * world

	// Keep the camera outside every pass of the tube -- back it off until it
	// clears the centreline by the radius plus the displacement -- and outside
	// the ellipsoid, so the silhouette formula holds.
	camera := cameraDirection(t).Scaled(extent186 * (1.5 + .4*sin(t)) * .075)
	for {
		nearest := math.Inf(1)
		for _, centre := range centersProbe {
			nearest = math.Min(nearest, camera.Minus(centre.Scaled(world)).Len())
		}
		if nearest > r*norm*(1+1.5*displacementFraction(t))*.075 && camera.Len() > c*1.05 {
			break
		}
		camera = camera.Scaled(1.05)
	}
	D := camera.Len()
	silhouetteH, silhouetteV := a/sqrt(D*D-c*c), b/sqrt(D*D-c*c)
	// Widest frame with its corners on the silhouette: (X, X/aspect) on the ellipse.
	tanH := fillMargin / sqrt(1/(silhouetteH*silhouetteH)+1/(aspectRatio*aspectRatio*silhouetteV*silhouetteV))
	tanH = math.Max(math.Tan(minFOV/2*pi/180), math.Min(math.Tan(maxFOV/2*pi/180), tanH))
	fov := 2 * math.Atan(tanH) * 180 / pi
	framed := func(p, camera geom.Vec) bool {
		d := p.Minus(camera)
		z := d.Dot(view)
		return z > 0 && abs(d.Dot(imageRight)/z) <= tanH && abs(d.Dot(imageUp)/z) <= tanH/aspectRatio
	}
	fmt.Printf("scale right %.2f up %.2f, camera/ellipsoid depth %.2f, fov: %.1f deg\n", rightScale, upScale, D/c, fov)
	path := func(t float64) geom.Vec {
		return reshaped(knot.Eval(t))
	}

	// Displacement is one warp of space applied to every vertex, not something
	// worked out per tube pass, so where passes of the tube cross, the coincident
	// points move together and the relief stays continuous across the crossing.
	// Each point moves along its own direction from the centre, by the same
	// amount everywhere -- so the texture is uniform over the whole knot rather
	// than fading toward the middle the way 186's distance-scaled version did --
	// and that amount is a share of the tube radius, so a thin tube isn't
	// crumpled by relief sized for a fat one. The direction is softly normalized:
	// within displacementCore of the centre, where x/|x| would flip, it tapers
	// to nothing instead of tearing the surface.
	amount := textureAmount(t)
	displacement := displacementFraction(t) * r * norm
	warp := func(p geom.Vec) geom.Vec {
		d := p.Scaled(1 / sqrt(p.Dot(p)+displacementCore*displacementCore))
		return p.Plus(d.Scaled(displacement * texture(amount, p.X, p.Y, p.Z, t)))
	}
	// The finished surface point (in world units) at ring uIndex of nU around
	// centre i of the centers/e1/e2 sampling.
	surface := func(centers, e1, e2 []geom.Vec, uIndex, nU, i int) geom.Vec {
		u := float64(uIndex) / float64(nU) * 2 * pi
		radial := e1[i].Scaled(cos(u)).Plus(e2[i].Scaled(sin(u)))
		return warp(centers[i].Plus(radial.Scaled(r)).Scaled(norm)).Scaled(.075)
	}

	// Square faces on average: a first pass over the finished surface -- reshaped
	// and displaced -- at a uniform 500 x 500, summing the edge lengths around
	// and along the tube, gives the ratio of nU to nV that makes the average face
	// square for the triangle budget.
	const pass = 500
	passCenters, passE1, passE2 := utils.TubeFrame(path, pass)
	around, along := 0.0, 0.0
	for uIndex := 0; uIndex < pass; uIndex++ {
		for vIndex := 0; vIndex < pass; vIndex++ {
			p := surface(passCenters, passE1, passE2, uIndex, pass, vIndex)
			around += surface(passCenters, passE1, passE2, uIndex+1, pass, vIndex).Minus(p).Len()
			along += surface(passCenters, passE1, passE2, uIndex, pass, (vIndex+1)%pass).Minus(p).Len()
		}
	}
	ratio := around / along
	nU := int(sqrt(float64(desiredTriangles)/2*ratio) + .5)
	if nU < 24 {
		nU = 24
	}
	nV := desiredTriangles / 2 / nU
	fmt.Printf("frame: %v, t: %v, tube radius: %v, centreline length: %v, reach: %v, around/along: %v, nU: %v, nV: %v\n",
		frameNumber, t, r, length, knotReach, ratio, nU, nV)

	centers, e1, e2 := utils.TubeFrame(path, nV)
	positions := make([][]geom.Vec, nU+1)
	radials := make([][]geom.Vec, nU+1)
	for uIndex := 0; uIndex <= nU; uIndex++ {
		positions[uIndex] = make([]geom.Vec, nV+1)
		radials[uIndex] = make([]geom.Vec, nV+1)
		u := float64(uIndex) / float64(nU) * 2 * pi
		for vIndex := 0; vIndex <= nV; vIndex++ {
			i := vIndex % nV
			positions[uIndex][vIndex] = surface(centers, e1, e2, uIndex, nU, i)
			radials[uIndex][vIndex] = e1[i].Scaled(cos(u)).Plus(e2[i].Scaled(sin(u)))
		}
	}

	focusPoint := geom.Vec{0, 0, 0}
	distance := math.Inf(1)
	inView, sampled := 0, 0
	for uIndex := 0; uIndex <= nU; uIndex += 4 {
		for vIndex := 0; vIndex <= nV; vIndex += 4 {
			distance = math.Min(distance, camera.Minus(positions[uIndex][vIndex]).Len())
			sampled++
			if framed(positions[uIndex][vIndex], camera) {
				inView++
			}
		}
	}
	fmt.Printf("vertices in frame: %.1f%%\n", 100*float64(inView)/float64(sampled))

	plyData, _ := os.Create(fmt.Sprintf("data/%v.data.ply", frameNumber))
	PlyDataBuffered := bufio.NewWriter(plyData)
	at := func(uIndex, vIndex int) geom.Vec {
		uIndex = (uIndex + nU) % nU
		vIndex = (vIndex + nV) % nV
		return positions[uIndex][vIndex]
	}
	// Winding: du x dv points out of the tube for this frame's u/v sense.
	outwardSign := 1.0
	if at(1, 0).Minus(at(0, 0)).Cross(at(0, 1).Minus(at(0, 0))).Dot(radials[0][0]) < 0 {
		outwardSign = -1
	}
	for uIndex := 0; uIndex <= nU; uIndex++ {
		for vIndex := 0; vIndex <= nV; vIndex++ {
			vertex := positions[uIndex][vIndex]
			normal, _ := at(uIndex+1, vIndex).Minus(at(uIndex-1, vIndex)).Cross(at(uIndex, vIndex+1).Minus(at(uIndex, vIndex-1))).Scaled(outwardSign).Unit()
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(vertex.X))
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(vertex.Y))
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(vertex.Z))
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(normal.X))
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(normal.Y))
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(normal.Z))
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(float64(uIndex)/float64(nU)))
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(float64(vIndex)/float64(nV)))
		}
	}
	index := func(uIndex, vIndex int) int32 {
		return int32(uIndex*(nV+1) + vIndex)
	}
	blendArray := []float32{}
	numFaces := 0
	outwardFaces := 0
	for vIndex := 0; vIndex < nV; vIndex++ {
		for uIndex := 0; uIndex < nU; uIndex++ {
			p := positions[uIndex][vIndex].Scaled(1 / .075)
			b := float32(blendValue(p, t))
			blendArray = append(blendArray, b, b, b)

			a, bb, c, d := index(uIndex, vIndex), index(uIndex+1, vIndex), index(uIndex, vIndex+1), index(uIndex+1, vIndex+1)
			faces := [2][3]int32{{a, bb, d}, {a, d, c}}
			if outwardSign < 0 {
				faces = [2][3]int32{{a, d, bb}, {a, c, d}}
			}
			// Where the radius outruns the centreline's curvature the sweep folds
			// inside out, as most of 186's did; count how much of it.
			pa, pb, pd := positions[uIndex][vIndex], positions[uIndex+1][vIndex], positions[uIndex+1][vIndex+1]
			if pb.Minus(pa).Cross(pd.Minus(pa)).Scaled(outwardSign).Dot(radials[uIndex][vIndex]) >= 0 {
				outwardFaces++
			}
			for _, face := range faces {
				binary.Write(PlyDataBuffered, binary.LittleEndian, byte(3))
				binary.Write(PlyDataBuffered, binary.LittleEndian, face)
				numFaces++
			}
		}
	}
	fmt.Printf("faces whose winding points out of the tube: %.1f%%\n", 100*float64(outwardFaces)/float64(numFaces/2))
	envmapArray := []float32{}
	for vIndex := 0; vIndex < envSize; vIndex++ {
		for uIndex := 0; uIndex < envSize; uIndex++ {
			u := float64(uIndex) / float64(envSize) * 2 * pi
			v := float64(vIndex) / float64(envSize) * pi
			power := 2 * pow(10, sin(5*t)/2+.5)
			envmapValue := float32(pow(sin(u/2), power) * pow(sin(v), power))
			envmapArray = append(envmapArray, envmapValue, envmapValue, envmapValue)
		}
	}

	fmt.Println("Mitsuba!")
	PlyDataBuffered.Flush()
	tmpl, _ := template.New("some template").Parse(`
ply
format binary_little_endian 1.0
element vertex {{ .NumVertices }}
property float32 x
property float32 y
property float32 z
property float32 nx
property float32 ny
property float32 nz
property float32 u
property float32 v
element face {{ .NumFaces }}
property list uint8 int32 vertex_index
end_header
`)
	plyHeader, _ := os.Create(fmt.Sprintf("data/%v.header.ply", frameNumber))
	tmpl.Execute(plyHeader, MeshType{(nU + 1) * (nV + 1), numFaces})
	envmap, _ := os.Create(fmt.Sprintf("data/%v.rgbe", frameNumber))
	rgbe.Encode(envmap, envSize, envSize, envmapArray)
	blend, _ := os.Create(fmt.Sprintf("data/%v.blend.rgbe", frameNumber))
	rgbe.Encode(blend, nU, nV, blendArray)
	sensorFile, _ := os.Create("sensor.xml")

	type sensor struct {
		Camera        geom.Vec
		LookAt        geom.Vec
		Distance      float64
		IntIOR        float64
		ExtIOR        float64
		G             float64
		Abbe          float64
		FilmThickness float64
		FilmIOR       float64
		Albedo        float64
		SigmaT        float64
		EnvX          float64
		EnvY          float64
		EnvZ          float64
		Width         int
		Height        int
		Samples       int
		FOV           float64
	}
	sensorTemplate, _ := template.New("some template").Parse(`
<scene version="2.0.0">
    <sensor type="thinlens" id="Camera-camera">
        <string name="fov_axis" value="larger"/>
        <float name="focus_distance" value="{{ .Distance }}"/>
        <float name="aperture_radius" value="0.0000000000001"/>
        <float name="fov" value="{{ .FOV }}"/>
        <transform name="to_world">
            <lookat origin="{{ .Camera.X }}, {{ .Camera.Y }}, {{ .Camera.Z }}" target="{{ .LookAt.X }}, {{ .LookAt.Y }}, {{ .LookAt.Z }}" up="0, 0, 1"/>
        </transform>

        <sampler type="multijitter">
            <integer name="sample_count" value="{{ .Samples }}"/>
        </sampler>

        <film type="hdrfilm" id="film">
            <integer name="width" value="{{ .Width }}"/>
            <integer name="height" value="{{ .Height }}"/>
            <rfilter type="lanczos"/>
        </film>
    </sensor>
    <emitter type="envmap" id="Area_002-light">
        <string name="filename" value="mitsuba.rgbe"/>
        <transform name="to_world">
            <rotate value="1, 0, 0" angle="{{ .EnvX }}"/>
            <rotate value="0, 1, 0" angle="{{ .EnvY }}"/>
            <rotate value="0, 0, 1" angle="{{ .EnvZ }}"/>
        </transform>
    </emitter>
        <integrator type="volpathmis">
            <integer name="max_depth" value="16"/>
        </integrator>
    <medium id="medium1" type="homogeneous">
        <float name="albedo" value="{{ .Albedo }}"/>
        <float name="sigma_t" value="{{ .SigmaT }}"/>
        <phase type="hg">
			<float name="g" value="{{ .G }}"/>
		</phase>
    </medium>
    <shape type="ply">
        <string name="filename" value="mitsuba.ply"/>
		<bsdf type="blendbsdf">
			<texture type="bitmap" name="weight">
				<string name="filename" value="mitsuba.blend.rgbe"/>
				<boolean name="raw" value="true"/>
			</texture>
			<bsdf type="thindielectric">
				<float name="film_thickness" value="{{ .FilmThickness }}"/>
				<float name="film_ior" value="{{ .FilmIOR }}"/>
				<float name="int_ior" value="{{ .IntIOR }}"/>
				<float name="ext_ior" value="{{ .ExtIOR }}"/>
				<float name="abbe" value="{{ .Abbe }}"/>
			</bsdf>
			<bsdf type="thindielectric">
				<float name="int_ior" value="{{ .IntIOR }}"/>
				<float name="ext_ior" value="{{ .ExtIOR }}"/>
				<float name="abbe" value="{{ .Abbe }}"/>
			</bsdf>
		</bsdf>
        <ref id="medium1" name="interior"/>
    </shape>
</scene>
`)

	// Index contrast in [1.0001, 1.01] -- low enough that a bare face reflects
	// almost nothing and the image is built from faint film reflections stacked
	// through many layers (see series187).
	intIor := 1 + pow(10, cos(7*t)-3)
	extIor := 1.0
	if frameNumber%2 == 1 {
		extIor = intIor
		intIor = 1.0
	}

	// Low interference order and a weak film: the frames that worked all sat in
	// the bottom half of the old [100, 750] nm and [1.1, 2.5] ranges.
	opticalThickness := 275 + 175*sin(11*t+4)   // n·d ∈ [100, 450] nm
	filmIor := 1.45 + .35*sin(9*t+3)            // ∈ [1.1, 1.8]
	filmThickness := opticalThickness / filmIor // d follows from the two

	// Target dispersion strength: subtle to noticeable "flint-like" fire,
	// staying out of the rutile-grade noisy tail.
	dispStrength := .02 + .005*sin(13*t+5)           // Δn ∈ [0.015, 0.025]
	abbe := (max(intIor, extIor) - 1) / dispStrength // derived, not swept

	sensorTemplate.Execute(sensorFile, sensor{
		camera,
		focusPoint,
		distance,
		intIor,
		extIor,
		cos(10*t) * .95,
		abbe,
		filmThickness,
		filmIor,
		sin(4*t)*.5 + .5,
		pow(10, sin(6*t)*4-4),
		cos(9*t) * 179,
		cos(11*t) * 179,
		cos(13*t) * 179,
		width,
		int(math.Round(float64(width) / aspectRatio)),
		samples,
		fov,
	})
	fmt.Printf("ior: %v, camera distance: %v, focus distance: %v\n", max(intIor, extIor), camera.Len(), distance)
}

func main() {
	frame := flag.Int("frame", 0, "Specify frame")
	maxFrames := flag.Int("maxframes", 256, "Max frames")
	desiredTriangles := flag.Int("desiredtriangles", 5000000, "The desired number of triangles to render")
	knotPath := flag.String("knot", "knot_789.rr/knot.final.vect", "ridgerunner .vect centreline")
	width := flag.Int("width", 6144, "Image width")
	samples := flag.Int("samples", 1024, "Samples per pixel")
	flag.Parse()
	dt := pi * 2 / float64(*maxFrames)
	renderSurfaces(*frame, dt, *desiredTriangles, *knotPath, *width, *samples)
}
