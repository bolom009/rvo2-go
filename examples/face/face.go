package main

import (
	"fmt"
	"math"
	"math/rand"
	"time"

	rvo "github.com/bolom009/rvo2-go"
	rl "github.com/gen2brain/raylib-go/raylib"
)

func main() {
	rand.Seed(time.Now().UnixNano())

	var (
		sim          = rvo.NewEmptyRVOSimulator()
		width  int32 = 800
		height int32 = 600
		camera       = rl.NewCamera2D(
			rl.NewVector2(0, 0), rl.NewVector2(float32(width/2)*-1, float32(height/2)*-1), 0, 1.0,
		)
	)

	setupScenario(sim)
	setPreferredVelocities(sim)

	rl.InitWindow(width, height, "Example: Face2Face")

	for {
		if rl.WindowShouldClose() {
			break
		}

		eventSystem(&camera)

		if sim.IsReachedGoal() {
			fmt.Printf("Goal \n ")
			break
		}

		updateVisualization(sim, &camera)

		setPreferredVelocities(sim)
		sim.DoStep()
	}

	rl.CloseWindow()
}

const (
	avoidanceScale  float32 = 8.0
	avoidanceRadius float32 = 15.0
	agentRadius             = 3.0
)

func setupScenario(sim *rvo.RVOSimulator) {
	sim.SetTimeStep(0.1)
	sim.SetAgentDefaults(2*avoidanceScale, 20, 10*avoidanceScale, 10.0, agentRadius, 0.1, &rvo.Vector2{}) // where is velocity property ?

	id, err := sim.AddDefaultAgent(rvo.NewVector2(100, 100))
	if err == nil {
		sim.SetAgentGoal(id, rvo.NewVector2(100, 100))
	}

	for i := 0; i < 10; i++ {
		id, err = sim.AddDefaultAgent(rvo.NewVector2(float32(100+(i+1)*6), 100))
		if err == nil {
			sim.SetAgentGoal(id, rvo.NewVector2(float32(100+(i+1)*6), 100))
		}

		id, err = sim.AddDefaultAgent(rvo.NewVector2(float32(100-(i+1)*6), 100))
		if err == nil {
			sim.SetAgentGoal(id, rvo.NewVector2(float32(100-(i+1)*6), 100))
		}
	}

	id2, err := sim.AddDefaultAgent(rvo.NewVector2(-100, -100))
	if err == nil {
		sim.SetAgentGoal(id2, rvo.NewVector2(200, 200))
	}
}

func updateVisualization(sim *rvo.RVOSimulator, camera *rl.Camera2D) {
	rl.BeginDrawing()
	rl.ClearBackground(rl.White)
	rl.BeginMode2D(*camera)

	for i := uint16(0); i < sim.GetNumAgents(); i++ {
		if !sim.GetAgentActive(i) {
			continue
		}

		rvoAgent := sim.GetAgent(i)

		rl.DrawCircle(int32(rvoAgent.Goal.X), int32(rvoAgent.Goal.Y), agentRadius, rl.Gray)
		rl.DrawLine(int32(rvoAgent.Position.X), int32(rvoAgent.Position.Y), int32(rvoAgent.Goal.X), int32(rvoAgent.Goal.Y), rl.Blue)

		color := rl.Red
		if i == 0 {
			color = rl.Green
		}
		rl.DrawCircle(int32(rvoAgent.Position.X), int32(rvoAgent.Position.Y), agentRadius, color)
	}

	rl.EndMode2D()
	rl.EndDrawing()
}

func setPreferredVelocities(sim *rvo.RVOSimulator) {
	numAgents := sim.GetNumAgents()

	for i := uint16(0); i < numAgents; i++ {
		if !sim.GetAgentActive(i) {
			continue
		}

		iPos := sim.GetAgentPosition(i)
		iGoal := sim.GetAgentGoal(i)
		if rvo.Abs(rvo.Sub(iPos, iGoal)) < 0.5 {
			sim.SetAgentPrefVelocity(i, rvo.NewVector2(0, 0))
			sim.SetAgentVelocity(i, rvo.NewVector2(0, 0))
			continue
		}

		iSpeed := sim.GetAgentMaxSpeed(i)

		goalDirection := rvo.Normalize(rvo.Sub(iGoal, iPos))
		preferredVelocity := rvo.MulOne(goalDirection, iSpeed)

		for j := uint16(0); j < numAgents; j++ {
			if i != j {
				if !sim.GetAgentActive(j) {
					continue
				}

				toOther := rvo.Sub(sim.GetAgentPosition(j), iPos)
				distance := rvo.Abs(toOther)
				if distance < avoidanceRadius {
					avoidance := rvo.MulOne(rvo.Normalize(toOther), iSpeed*(avoidanceRadius-distance)/avoidanceRadius)
					preferredVelocity = rvo.Sub(preferredVelocity, avoidance)
				}
			}
		}

		// Limit the velocity to not exceed max speed
		if rvo.Abs(preferredVelocity) > iSpeed {
			preferredVelocity = rvo.MulOne(rvo.Normalize(preferredVelocity), iSpeed)
		}

		sim.SetAgentPrefVelocity(i, preferredVelocity)
		sim.SetAgentVelocity(i, preferredVelocity)

		//iPos.X += preferredVelocity.X
		//iPos.Y += preferredVelocity.Y
		//
		//sim.SetAgentPosition(i, iPos)

		//goalVector := sim.GetAgentGoalVector(i)
		//if rvo.Sqr(goalVector) > 1 {
		//	goalVector = rvo.Normalize(goalVector)
		//}
		//
		//sim.SetAgentPrefVelocity(i, goalVector)
		//sim.SetAgentVelocity(i, goalVector)
	}
}

func eventSystem(camera *rl.Camera2D) {
	mouseWorldPos := rl.GetScreenToWorld2D(rl.GetMousePosition(), *camera)

	wheel := rl.GetMouseWheelMove()
	if wheel != 0 {
		// Set the offset to where the mouse is
		camera.Offset = rl.GetMousePosition()

		// Set the target to match, so that the camera maps the world space point
		// under the cursor to the screen space point under the cursor at any zoom
		camera.Target = mouseWorldPos

		// Zoom increment
		scaleFactor := 1.0 + (0.25 * math.Abs(float64(wheel)))
		if wheel < 0 {
			scaleFactor = 1.0 / scaleFactor
		}

		camera.Zoom = rl.Clamp(camera.Zoom*float32(scaleFactor), 0.125, 64.0)
	}

	if rl.IsMouseButtonDown(rl.MouseRightButton) {
		delta := rl.GetMouseDelta()
		delta = rl.Vector2Scale(delta, -1.0/camera.Zoom)
		camera.Target = rl.Vector2Add(camera.Target, delta)
	}
}
