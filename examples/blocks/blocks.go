package main

import (
	"fmt"

	rvo "github.com/bolom009/rvo2-go"
	rl "github.com/gen2brain/raylib-go/raylib"
)

func setupScenario(sim *rvo.RVOSimulator) {
	sim.SetTimeStep(0.25)
	sim.SetAgentDefaults(15.0, 10, 5.0, 5.0, 1.5, 0.1, &rvo.Vector2{}) // where is velocity property ?

	for i := 0; i < 5; i++ {
		for j := 0; j < 5; j++ {
			id1, _ := sim.AddDefaultAgent(&rvo.Vector2{X: 55.0 + float64(i)*10.0, Y: 55.0 + float64(j)*10.0})
			sim.SetAgentGoal(id1, &rvo.Vector2{X: -75.0, Y: -75.0})

			id2, _ := sim.AddDefaultAgent(&rvo.Vector2{X: -55.0 - float64(i)*10.0, Y: 55.0 + float64(j)*10.0})
			sim.SetAgentGoal(id2, &rvo.Vector2{X: 75.0, Y: -75.0})

			id3, _ := sim.AddDefaultAgent(&rvo.Vector2{X: 55.0 + float64(i)*10.0, Y: -55.0 - float64(j)*10.0})
			sim.SetAgentGoal(id3, &rvo.Vector2{X: -75.0, Y: 75.0})

			id4, _ := sim.AddDefaultAgent(&rvo.Vector2{X: -55.0 - float64(i)*10.0, Y: -55.0 - float64(j)*10.0})
			sim.SetAgentGoal(id4, &rvo.Vector2{X: 75.0, Y: 75.0})
		}
	}

	obstacle1 := []*rvo.Vector2{
		rvo.NewVector2(-10.0, 40.0),
		rvo.NewVector2(-40.0, 40.0),
		rvo.NewVector2(-40.0, 10.0),
		rvo.NewVector2(-10.0, 10.0),
	}
	obstacle2 := []*rvo.Vector2{
		rvo.NewVector2(10.0, 40.0),
		rvo.NewVector2(10.0, 10.0),
		rvo.NewVector2(40.0, 10.0),
		rvo.NewVector2(40.0, 40.0),
	}
	obstacle3 := []*rvo.Vector2{
		rvo.NewVector2(10.0, -40.0),
		rvo.NewVector2(40.0, -40.0),
		rvo.NewVector2(40.0, -10.0),
		rvo.NewVector2(10.0, -10.0),
	}
	obstacle4 := []*rvo.Vector2{
		rvo.NewVector2(-10.0, -40.0),
		rvo.NewVector2(-10.0, -10.0),
		rvo.NewVector2(-40.0, -10.0),
		rvo.NewVector2(-40.0, -40.0),
	}

	sim.AddObstacle(obstacle1)
	sim.AddObstacle(obstacle2)
	sim.AddObstacle(obstacle3)
	sim.AddObstacle(obstacle4)
	sim.ProcessObstacles()

	fmt.Printf("Simulation has %v agents and %v obstacle vertices in it.\n", sim.GetNumAgents(), sim.GetNumObstacleVertices())
	fmt.Printf("Running Simulation...\n\n")
}

func reachedGoal(sim *rvo.RVOSimulator, distance float64) bool {
	/* Check if all agents have reached their goals. */
	for i := 0; i < sim.GetNumAgents(); i++ {
		position := sim.GetAgentPosition(i)
		if rvo.Sqr(rvo.Sub(position, sim.GetAgentGoal(i))) > distance {
			return false
		}
	}

	return true
}

func setPreferredVelocities(sim *rvo.RVOSimulator) {
	for i := 0; i < sim.GetNumAgents(); i++ {
		goalVector := sim.GetAgentGoalVector(i)
		if rvo.Sqr(goalVector) > 1 {
			goalVector = rvo.Normalize(goalVector)
		}

		sim.SetAgentPrefVelocity(i, goalVector)
	}
}

func updateVisualization(sim *rvo.RVOSimulator, camera rl.Camera2D) {
	rl.BeginDrawing()
	rl.ClearBackground(rl.White)
	rl.BeginMode2D(camera)

	for i := 0; i < sim.GetNumObstacles(); i++ {
		rvoObstacle := sim.GetObstacle(i)
		for i, vec := range rvoObstacle {
			if i == 0 {
				continue
			}

			pos1 := rvoObstacle[i-1]
			pos2 := vec

			rl.DrawLine(int32(pos1.X), int32(pos1.Y), int32(pos2.X), int32(pos2.Y), rl.Blue)
		}

		if len(rvoObstacle) > 2 {
			pos1 := rvoObstacle[0]
			pos2 := rvoObstacle[len(rvoObstacle)-1]

			rl.DrawLine(int32(pos1.X), int32(pos1.Y), int32(pos2.X), int32(pos2.Y), rl.Blue)
		}
	}

	for i := 0; i < sim.GetNumAgents(); i++ {
		rvoAgent := sim.GetAgent(i)
		rl.DrawCircle(int32(rvoAgent.Goal.X), int32(rvoAgent.Goal.Y), 1.5, rl.Gray)
		rl.DrawCircle(int32(rvoAgent.Position.X), int32(rvoAgent.Position.Y), 1.5, rl.Red)
	}

	rl.EndMode2D()
	rl.EndDrawing()
}

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

	rl.InitWindow(width, height, "Example: Blocks")

	for {
		if rl.WindowShouldClose() {
			break
		}

		if reachedGoal(sim, 400.0) {
			break
		}

		updateVisualization(sim, camera)

		setPreferredVelocities(sim)
		sim.DoStep()
	}

	rl.CloseWindow()
}
