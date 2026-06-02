//go:build test1
// +build test1

package main

import (
	"fmt"
	"math"
	"os"
	"strconv"

	"github.com/rrothenb/pbr/pkg/camera"
	"github.com/rrothenb/pbr/pkg/env"
	"github.com/rrothenb/pbr/pkg/geom"
	"github.com/rrothenb/pbr/pkg/material"
	"github.com/rrothenb/pbr/pkg/render"
	"github.com/rrothenb/pbr/pkg/rgb"
	"github.com/rrothenb/pbr/pkg/surface"
)

func uv2xyz(u, v, t float64, texture func(u, v, t float64) float64) geom.Vec {
	a := texture(u, v, t)
	return geom.Vec{
		math.Sin(v/2.0) * math.Cos(u) * a,
		math.Sin(v/2.0) * math.Sin(u) * a,
		math.Cos(v/2.0) * a,
	}
}

func index2radians(index, n int) float64 {
	return float64(index) / float64(n) * math.Pi * 2
}

func texture(u, v, t float64) float64 {
	a := math.Sin(3*u + 5*v + 0.75*math.Sin(.1+3*t)*math.Sin(2*u+0.75*math.Sin(.2+5*t)*math.Sin(3*u)) + 0.75*math.Sin(.3+7*t)*math.Sin(7*v+0.75*math.Sin(.4+11*t)*math.Sin(5*v)) + 0.75*math.Sin(.5+13*t)*math.Sin(11*u+13*v) + 0.75*math.Sin(.6+17*t)*math.Sin(17*u-5*v) + 0.75*math.Sin(.7+19*t)*math.Sin(23*v-11*u))
	return 1.0 - .075*a*a*a*a
}

func renderFrame(frameNumber int, dt float64, surfaces []render.Surface) {
	n := 500
	t := float64(frameNumber) * dt
	for vIndex := 0; vIndex < n; vIndex++ {
		for uIndex := 0; uIndex < n; uIndex++ {
			topRight := uv2xyz(index2radians(uIndex, n), index2radians(vIndex, n), t, texture).Scaled(.075)
			topLeft := uv2xyz(index2radians(uIndex+1, n), index2radians(vIndex, n), t, texture).Scaled(.075)
			botRight := uv2xyz(index2radians(uIndex, n), index2radians(vIndex+1, n), t, texture).Scaled(.075)
			botLeft := uv2xyz(index2radians(uIndex+1, n), index2radians(vIndex+1, n), t, texture).Scaled(.075)
			roughness := math.Pow(texture(index2radians(uIndex, n), index2radians(vIndex, n), t), 30)*.35 + .05
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

	c := camera.NewSLR().MoveTo(geom.Vec{.4 * math.Sin(t), .4 * math.Sin(t), -.5 * math.Cos(t)}).LookAt(geom.Vec{0, 0, 0})
	c.FStop = 32
	s := surface.NewTree(surfaces...)
	e := env.NewGradient(rgb.Black, rgb.Energy{1000, 1000, 1000}, 7)

	scene := render.NewScene(c, s, e)
	framePath := fmt.Sprintf("images/%04v.png", frameNumber)
	err := render.Iterative(scene, framePath, 384, 384, 8, true)
	if err != nil {
		fmt.Fprintf(os.Stderr, "\nError: %v\n", err)
	}
}

func main() {
	floor := surface.UnitCube(material.Plastic(1, 1, 1, 0.05))
	floor.Shift(geom.Vec{0, -3, 0}).Scale(geom.Vec{10, 0.1, 10})
	rightWall := surface.UnitCube(material.Plastic(.4, .7, 1, 0.05))
	rightWall.Shift(geom.Vec{-3, 0, 0}).Scale(geom.Vec{0.1, 10, 10})
	leftWall := surface.UnitCube(material.Plastic(1, .7, .4, 0.05))
	leftWall.Shift(geom.Vec{3, 0, 0}).Scale(geom.Vec{0.1, 10, 10})
	frameNumber, _ := strconv.Atoi(os.Args[1])
	dt := math.Pi * 2 / 2000
	var surfaces []render.Surface
	surfaces = append(surfaces, floor, rightWall, leftWall)
	renderFrame(frameNumber, dt, surfaces)
}
