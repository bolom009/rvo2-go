package main

import (
	"fmt"
	"math"

	rvo "github.com/bolom009/rvo2-go"
	rl "github.com/gen2brain/raylib-go/raylib"
)

func main() {
	var (
		sim          = rvo.NewEmptyRVOSimulator()
		width  int32 = 800
		height int32 = 600
		camera       = rl.NewCamera2D(
			rl.NewVector2(0, 0), rl.NewVector2(float32(width/2)*-1, float32(height/2)*-1), 0, 1.0,
		)
	)

	setupScenario(sim)

	rl.InitWindow(width, height, "Example: Circle")

	for {
		if rl.WindowShouldClose() {
			break
		}

		if sim.IsReachedGoal() {
			fmt.Printf("Goal \n ")
			break
		}

		updateVisualization(sim, camera)

		setPreferredVelocities(sim)
		sim.DoStep()
	}

	rl.CloseWindow()
}

func setupScenario(sim *rvo.RVOSimulator) {
	sim.SetTimeStep(0.25)
	sim.SetAgentDefaults(15.0, 10, 10.0, 10.0, 1.5, 0.1, &rvo.Vector2{}) // where is velocity property ?

	var (
		agentNum         = 250
		radius   float32 = 200
	)
	for i := 0; i < agentNum; i++ {
		position := rvo.MulOne(rvo.NewVector2(
			float32(math.Cos(float64(i)*2.0*math.Pi/float64(agentNum))),
			float32(math.Sin(float64(i)*2.0*math.Pi/float64(agentNum))),
		), radius)

		id, err := sim.AddDefaultAgent(position)
		if err == nil {
			sim.SetAgentGoal(id, rvo.MulOne(position, -1))
		}
	}
}

func updateVisualization(sim *rvo.RVOSimulator, camera rl.Camera2D) {
	rl.BeginDrawing()
	rl.ClearBackground(rl.White)
	rl.BeginMode2D(camera)

	for i := uint16(0); i < sim.GetNumAgents(); i++ {
		if !sim.GetAgentActive(i) {
			continue
		}

		rvoAgent := sim.GetAgent(i)

		rl.DrawCircle(int32(rvoAgent.Goal.X), int32(rvoAgent.Goal.Y), 1.5, rl.Gray)
		rl.DrawCircle(int32(rvoAgent.Position.X), int32(rvoAgent.Position.Y), 1.5, rl.Red)
	}

	rl.EndMode2D()
	rl.EndDrawing()
}

func setPreferredVelocities(sim *rvo.RVOSimulator) {
	for i := uint16(0); i < sim.GetNumAgents(); i++ {
		if !sim.GetAgentActive(i) {
			continue
		}

		goalVector := sim.GetAgentGoalVector(i)
		if rvo.Sqr(goalVector) > 1 {
			goalVector = rvo.Normalize(goalVector)
		}

		sim.SetAgentPrefVelocity(i, goalVector)
	}
}
