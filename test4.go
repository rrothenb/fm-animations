//go:build ignore

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

func strength(x float64) float64 {
	return math.Sin(x)*.75 + .85
}

func texture(u, v, t float64) float64 {
	return math.Sin(
		3*u + 5*v + strength(.1+2*t)*math.Sin(
			2*u+strength(.2+3*t)*math.Sin(3*u)) + strength(.3+5*t)*math.Sin(
			7*v+strength(.4+7*t)*math.Sin(5*v)) + strength(.5+11*t)*math.Sin(
			11*u+13*v) + strength(.6+13*t)*math.Sin(17*u-5*v) + strength(.7+17*t)*math.Sin(23*v-11*u))
}

func radius(u, v, t float64) float64 {
	return 1.0 + .075*math.Pow(math.Pow(texture(u, v, t), 2), 3)
}

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
				if time.Now().UnixNano()-start > int64(maxSeconds)*1e9 {
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
	a := math.Pow(texture(u, v, e.t), 6) * 750
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

func renderFrame(frameNumber int, dt float64, surfaces []render.Surface) {
	pixels := 800
	t := float64(frameNumber) * dt
	cameraLoc := geom.Vec{math.Sin(t), math.Sin(t), math.Cos(t)}.Scaled(.3).Plus(geom.Vec{0.0, 0.1, -.1})
	unitCameraLoc, _ := cameraLoc.Unit()
	focusPoint := unitCameraLoc.Scaled(.075)
	c := camera.NewSLR().MoveTo(cameraLoc).LookAt(focusPoint)
	c.FStop = 32
	distance := cameraLoc.Minus(focusPoint).Len() - c.Lens
	n := int(float64(pixels) / distance / 2)
	if n > 2000 {
		n = 2000
	}
	maxSeconds := int(float64(pixels) / distance / 10)
	fmt.Printf("distance from center: %v, distance from focal point: %v, n: %v, maxSeconds: %v", cameraLoc.Len(), distance, n, maxSeconds)
	for vIndex := 0; vIndex < n; vIndex++ {
		for uIndex := 0; uIndex < n; uIndex++ {
			topRight := uv2xyz(index2radians(uIndex, n), index2radians(vIndex, n), t, radius).Scaled(.075)
			topLeft := uv2xyz(index2radians(uIndex+1, n), index2radians(vIndex, n), t, radius).Scaled(.075)
			botRight := uv2xyz(index2radians(uIndex, n), index2radians(vIndex+1, n), t, radius).Scaled(.075)
			botLeft := uv2xyz(index2radians(uIndex+1, n), index2radians(vIndex+1, n), t, radius).Scaled(.075)
			fm := texture(index2radians(uIndex, n), index2radians(vIndex, n), t)
			fm4 := math.Pow(fm, 4)
			roughness := fm4*.09 + .01
			metalness := (1-fm4)*.8 + .1
			specularity := fm4*.3 + .1
			color := rgb.Energy{
				1,
				.5 + .05*fm,
				.25 + .15*math.Sin(4*fm)}.Scaled(fm4).Plus(rgb.Energy{.8, .8, .8}.Scaled(1 - fm4))
			m := &material.Uniform{
				Color:       color,
				Metalness:   metalness,
				Roughness:   roughness,
				Specularity: specularity,
			}
			if vIndex != 0 {
				surfaces = append(surfaces,
					surface.NewTriangle(topRight, botLeft, topLeft, m))
			}
			if vIndex != n-1 {
				surfaces = append(surfaces,
					surface.NewTriangle(topRight, botRight, botLeft, m))
			}
		}
	}

	//surfaces = append(surfaces, surface.UnitSphere(material.Mirror(.01)).Scale(geom.Vec{.1,.1,.1}))

	s := surface.NewTree(surfaces...)
	//e := env.NewGradient(rgb.Black, rgb.Energy{1000, 1000, 1000}, 7)
	e := &FMEnv{t}

	scene := render.NewScene(c, s, e)
	framePath := fmt.Sprintf("images/%04v.png", frameNumber)
	err := Iterative(scene, framePath, pixels, pixels, 8, true, maxSeconds)
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
	dt := math.Pi * 2 / 2500
	var surfaces []render.Surface
	//surfaces = append(surfaces, floor, rightWall, leftWall)
	renderFrame(frameNumber, dt, surfaces)
}
