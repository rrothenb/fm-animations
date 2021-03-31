package main

import (
	"fmt"
	"golang.org/x/text/language"
	"golang.org/x/text/message"
	"image"
	"image/png"
	"math"
	"os"
	"os/signal"
	"strconv"
	"syscall"
	"time"

	"github.com/hunterloftis/pbr/pkg/camera"
	"github.com/hunterloftis/pbr/pkg/geom"
	"github.com/hunterloftis/pbr/pkg/material"
	"github.com/hunterloftis/pbr/pkg/render"
	"github.com/hunterloftis/pbr/pkg/rgb"
	"github.com/hunterloftis/pbr/pkg/surface"
)

func writePng(filename string, im image.Image) error {
	out, err := os.Create(filename)
	if err != nil {
		return err
	}
	defer out.Close()
	return png.Encode(out, im)
}

func Iterative(scene *render.Scene, file string, width, height, depth int, direct bool, maxSeconds int) error {
	kill := make(chan os.Signal, 2)
	signal.Notify(kill, os.Interrupt, syscall.SIGTERM)

	frame := scene.Render(width, height, depth, direct)
	defer frame.Stop()
	ticker := time.NewTicker(6 * time.Second) // 10 .s = 1 minute, 100 .s = 1 hr
	defer ticker.Stop()

	start := time.Now().UnixNano()
	max := 0
	fmt.Printf("\nRendering %v (Ctrl+C to end)", file)

	for frame.Active() {
		select {
		case <-kill:
			frame.Stop()
		case <-ticker.C:
			if sample, n := frame.Sample(); n > max {
				max = n
				fmt.Print(".")
				if err := writePng(file, sample.Image()); err != nil {
					return err
				}
				if time.Now().UnixNano() - start > int64(maxSeconds) * 1e9 {
					frame.Stop()
				}
			}
		}
	}

	stop := time.Now().UnixNano()
	sample, _ := frame.Sample()
	total := sample.Total()
	p := message.NewPrinter(language.English)
	secs := float64(stop-start) / 1e9
	sps := math.Round(float64(total) / secs) // TODO: rename to pixels/sec for clarity
	p.Printf("\n%v samples in %.1f seconds (%.0f samples/sec)\n", total, secs, sps)

	return nil
}

type FMEnv struct {
	t float64
}

func (e *FMEnv) At(dir geom.Dir) rgb.Energy {
	u := math.Atan2(dir.X, -dir.Z)
	v := math.Acos(dir.Y)
	a := math.Pow(texture(u, v, e.t), 4)*500
	return rgb.Energy{
		X: a,
		Y: a,
		Z: a,
	}
}

func uv2xyz(u, v, t float64, radius func(u, v, t float64) float64) geom.Vec {
	a := radius(u, v, t)
	return geom.Vec{
		math.Sin(v/2.0) * math.Cos(u) * a,
		math.Sin(v/2.0) * math.Sin(u) * a,
		math.Cos(v/2.0) * a,
	}
}

func index2radians(index, n int) float64 {
	return float64(index) / float64(n) * math.Pi * 2
}

func strength(x float64) float64 {
	return math.Sin(x)*.25 + .5
}

func texture(u, v, t float64) float64 {
	return math.Sin(
		3*u + 5*v + strength(.1+2*t)*math.Sin(
			2*u+strength(.2+3*t)*math.Sin(3*u)) + strength(.3+5*t)*math.Sin(
			7*v+strength(.4+7*t)*math.Sin(5*v)) + strength(.5+11*t)*math.Sin(
			11*u+13*v) + strength(.6+13*t)*math.Sin(17*u-5*v) + strength(.7+17*t)*math.Sin(23*v-11*u))
}

func radius(u, v, t float64) float64 {
	return 1.0 + .05*math.Pow(texture(u, v, t), 20)
}

func renderFrame(frameNumber int, dt float64, surfaces []render.Surface) {
	t := float64(frameNumber) * dt
	n := 1000
	for vIndex := 0; vIndex < n; vIndex++ {
		for uIndex := 0; uIndex < n; uIndex++ {
			topRight := uv2xyz(index2radians(uIndex, n), index2radians(vIndex, n), t, radius).Scaled(.075)
			topLeft := uv2xyz(index2radians(uIndex+1, n), index2radians(vIndex, n), t, radius).Scaled(.075)
			botRight := uv2xyz(index2radians(uIndex, n), index2radians(vIndex+1, n), t, radius).Scaled(.075)
			botLeft := uv2xyz(index2radians(uIndex+1, n), index2radians(vIndex+1, n), t, radius).Scaled(.075)
			roughness := math.Pow(texture(index2radians(uIndex, n), index2radians(vIndex, n), t), 4)*.09 + .01
			if vIndex != 0 {
				surfaces = append(surfaces,
					surface.NewTriangle(topRight, botLeft, topLeft, material.Mirror(roughness)))
			}
			if vIndex != n-1 {
				surfaces = append(surfaces,
					surface.NewTriangle(topRight, botRight, botLeft, material.Mirror(roughness)))
			}
		}
	}

	//surfaces = append(surfaces, surface.UnitSphere(material.Mirror(.01)).Scale(geom.Vec{.1,.1,.1}))

	c := camera.NewSLR().MoveTo(geom.Vec{.4 * math.Sin(t), .4 * math.Sin(t), -.5 * math.Cos(t)}).LookAt(geom.Vec{0, 0, 0})
	c.FStop = 32
	s := surface.NewTree(surfaces...)
	//e := env.NewGradient(rgb.Black, rgb.Energy{1000, 1000, 1000}, 7)
	e := &FMEnv{t}

	scene := render.NewScene(c, s, e)
	framePath := fmt.Sprintf("images/%04v.png", frameNumber)
	err := Iterative(scene, framePath, 1000, 1000, 6, true, 300)
	if err != nil {
		fmt.Fprintf(os.Stderr, "\nError: %v\n", err)
	}
}

func main() {
	/*
		floor := surface.UnitCube(material.Plastic(1, 1, 1, 0.05))
		floor.Shift(geom.Vec{0, -3, 0}).Scale(geom.Vec{10, 0.1, 10})
		rightWall := surface.UnitCube(material.Plastic(.4, .7, 1, 0.05))
		rightWall.Shift(geom.Vec{-3, 0, 0}).Scale(geom.Vec{0.1, 10, 10})
		leftWall := surface.UnitCube(material.Plastic(1, .7, .4, 0.05))
		leftWall.Shift(geom.Vec{3, 0, 0}).Scale(geom.Vec{0.1, 10, 10})
	*/
	frameNumber, _ := strconv.Atoi(os.Args[1])
	dt := math.Pi * 2 / 2000
	var surfaces []render.Surface
	//surfaces = append(surfaces, floor, rightWall, leftWall)
	renderFrame(frameNumber, dt, surfaces)
}
