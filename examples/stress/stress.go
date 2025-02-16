package main

import (
	"fmt"
	rl "github.com/gen2brain/raylib-go/raylib"
	"github.com/knadh/profiler"
	"math"
	"math/rand"
	"runtime"
	"slices"
	"time"

	rvo "github.com/bolom009/rvo2-go"
)

type Obstacle []*rvo.Vector2

// AVG: 372.028µs
func main() {
	runtime.GOMAXPROCS(1)

	rand.Seed(time.Now().UnixNano())

	var (
		sim          = rvo.NewEmptyRVOSimulator()
		width  int32 = 800
		height int32 = 600
		camera       = rl.NewCamera2D(
			rl.NewVector2(0, 0), rl.NewVector2(float32(width/2)*-1, float32(height/2)*-1), 0, 1.0,
		)
		p                         = profiler.New(profiler.Conf{}, profiler.Cpu, profiler.Mem)
		obstacles                 = make([]Obstacle, 0)
		totalFrames time.Duration = 0
		stepsCount                = 0
		frameTime   time.Duration = 0
	)

	_ = frameTime

	for i := 0; i < 20; i++ {
		shift := float32(i * 100)
		rX := float32(randRange(-500, 500))
		rY := float32(0) //float32(randRange(-500, 500))
		obstacles = append(obstacles, Obstacle{
			&rvo.Vector2{X: 0 + shift + rX, Y: 0 + shift + rY},
			&rvo.Vector2{X: 0 + shift + rX, Y: 70 + shift + rY},
			&rvo.Vector2{X: 70 + shift + rX, Y: 70 + shift + rY},
			&rvo.Vector2{X: 70 + shift + rX, Y: 0 + shift + rY},
		})
	}

	setupScenario(sim, obstacles)
	setPreferredVelocities(sim)

	rl.InitWindow(width, height, "Example: Face2Face")

	ticker := time.NewTicker(time.Second / 30)
	titleTicker := time.NewTicker(time.Millisecond * 500)
	stopTicker := time.NewTimer(time.Second * 20)
	p.Start()
	for {
		if rl.WindowShouldClose() {
			p.Stop()
			break
		}

		if sim.IsReachedGoal() {
			fmt.Printf("Goal \n ")
			break
		}

		eventSystem(&camera)
		updateVisualization(sim, obstacles, &camera)

		select {
		case <-ticker.C:
			t := time.Now()
			setPreferredVelocities(sim)
			sim.DoStep()
			frameTime = time.Since(t)
			totalFrames += frameTime
			stepsCount++
		case <-titleTicker.C:
			rl.SetWindowTitle(fmt.Sprintf("Example: Face2Face - %s", frameTime.String()))
		case <-stopTicker.C:
			fmt.Printf("AVG: %v\n", (totalFrames / time.Duration(stepsCount)).String())
			p.Stop()
			return
		}
	}

	rl.CloseWindow()
}

const (
	avoidanceScale float32 = 8.0
	agentRadius            = 5.0
)

func setupScenario(sim *rvo.RVOSimulator, obstacles []Obstacle) {
	sim.SetTimeStep(0.1)
	sim.SetAgentDefaults(2*avoidanceScale, 20, 10*avoidanceScale, 3.0, agentRadius, 7, &rvo.Vector2{}) // where is velocity property ?

	for _, obstacle := range obstacles {
		slices.Reverse(obstacle)
		_, _ = sim.AddObstacle(obstacle)
	}
	sim.ProcessObstacles()

	pp := generateSpiralPositions(500, 4, math.Pi, rvo.Vector2{100, 100})
	for _, p := range pp {
		agentId, _ := sim.AddDefaultAgent(rvo.NewVector2(p.X, p.Y))

		agent := sim.GetAgent(agentId)
		agent.PrefVelocity = rvo.NewVector2(0, 0)
		agent.Goal = rvo.NewVector2(20000, 20000)
	}
}

func updateVisualization(sim *rvo.RVOSimulator, obstacles []Obstacle, camera *rl.Camera2D) {
	rl.BeginDrawing()
	rl.ClearBackground(rl.White)
	rl.BeginMode2D(*camera)

	for _, obstacle := range obstacles {
		if len(obstacle) < 2 {
			continue
		}

		for i, pos := range obstacle[1:] {
			lastPos := obstacle[i]

			rl.DrawLine(int32(lastPos.X), int32(lastPos.Y), int32(pos.X), int32(pos.Y), rl.Orange)
		}

		if len(obstacle) > 2 {
			pos1 := obstacle[0]
			pos2 := obstacle[len(obstacle)-1]

			rl.DrawLine(int32(pos1.X), int32(pos1.Y), int32(pos2.X), int32(pos2.Y), rl.Orange)
		}
	}

	for i := uint16(0); i < sim.GetNumAgents(); i++ {
		if !sim.GetAgentActive(i) {
			continue
		}

		rvoAgent := sim.GetAgent(i)

		rl.DrawCircle(int32(rvoAgent.Goal.X), int32(rvoAgent.Goal.Y), agentRadius, rl.Gray)

		// draw obstacle detection range
		//rl.DrawCircleLines(int32(rvoAgent.Position.X), int32(rvoAgent.Position.Y), rvoAgent.ObstacleRangeSq, rl.Blue)

		// draw goal line
		//rl.DrawLine(int32(rvoAgent.Position.X), int32(rvoAgent.Position.Y), int32(rvoAgent.Goal.X), int32(rvoAgent.Goal.Y), rl.Blue)

		color := rl.Red
		if i == 0 {
			color = rl.Green
		}
		rl.DrawCircle(int32(rvoAgent.Position.X), int32(rvoAgent.Position.Y), agentRadius, color)
	}

	rl.EndMode2D()
	rl.EndDrawing()
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

func setPreferredVelocities(sim *rvo.RVOSimulator) {
	for _, agent := range sim.GetAgents() {
		if !agent.IsActive() {
			continue
		}

		iPos := agent.Position
		iGoal := agent.Goal
		if rvo.Abs(rvo.Sub(iPos, iGoal)) < 0.5 {
			agent.PrefVelocity = rvo.NewVector2(0, 0)
			continue
		}

		iSpeed := agent.MaxSpeed

		goalDirection := rvo.Normalize(rvo.Sub(iGoal, iPos))
		preferredVelocity := rvo.MulOne(goalDirection, iSpeed)
		if rvo.Abs(preferredVelocity) > iSpeed {
			preferredVelocity = rvo.MulOne(rvo.Normalize(preferredVelocity), iSpeed)
		}

		agent.PrefVelocity = preferredVelocity
	}
}

func generateSpiralPositions(numPoints int, radiusStep, angleStep float32, offset rvo.Vector2) []*rvo.Vector2 {
	positions := make([]*rvo.Vector2, numPoints)

	angleStep = angleStep / (float32(numPoints) / 10)

	for i := 0; i < numPoints; i++ {
		// Рассчитываем угол в радианах
		angle := angleStep * float32(i)

		// Увеличиваем радиус на каждом шаге
		radius := radiusStep * float32(i)

		// Рассчитываем координаты по спирали
		x := offset.X + radius*float32(math.Cos(float64(angle)))
		y := offset.Y + radius*float32(math.Sin(float64(angle)))

		// Создаем вектор с полученными координатами
		positions[i] = &rvo.Vector2{X: x, Y: y}
	}

	return positions
}

func randRange(min, max int) int {
	return rand.Intn(max-min) + min
}
