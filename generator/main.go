package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"git.solver4all.com/azaryc2s/op"
	"git.solver4all.com/azaryc2s/op/tsp"
	"io/ioutil"
	"log"
	"math"
	"math/rand"
	"regexp"
	"time"
)

var prices op.ArrayStringFlags
var nodes op.ArrayIntFlags
var tmaxA op.ArrayFloatFlags

func main() {
	flag.Var(&prices, "prices", "List of price-generation strategies")
	flag.Var(&nodes, "n", "List of number of nodes")
	flag.Var(&tmaxA, "tmax", "List of maximum lengths of the route as value a: 0 < a < 1 which is a portion of the tsp-length")
	name := flag.String("name", "zarychta", "Name for the instance")
	//comment := flag.String("comment", "Zarychta generated OP-Instance", "Comment for the instances")
	//dimension := flag.Int("n", 0, "List of number of nodes")
	count := flag.Int("count", 10, "Number of instances per combination")
	xTo := flag.Int("x", 10000, "Max value on the x-axis")
	yTo := flag.Int("y", 10000, "Max value on the y-axis")
	w := flag.String("w", "EUC_2D", "EDGE_WEIGHT_TYPE - how the distance between nodes is calculated.")
	//priceTo := flag.Int("price", 0, "Max price for a node")
	calcTSP := flag.Bool("tsp", true, "Whether to calculate the tsp-route or not (needs gurobi configured and could take a while for bigger instances)")
	//tmaxA := flag.Float64("tmax", 0.5, "Maximum length of the route as value a: 0 < a < 1 which is a portion of the tsp-length")

	flag.Parse()

	for l := 0; l < *count; l++ {
		var tmax, tspLength int
		rand.Seed(time.Now().UnixNano())
		for i := 0; i < len(nodes); i++ {
			n := nodes[i]
			coordinatesArray := make([][]float64, n)
			edgeWeights := make([][]int, n)
			for node := 0; node < n; node++ {
				x := rand.Intn(*xTo)
				y := rand.Intn(*yTo)
				coordinatesArray[node] = []float64{float64(x), float64(y)}
				edgeWeights[node] = make([]int, n)
				for node2 := 0; node2 < node; node2++ {
					xDist := coordinatesArray[node][0] - coordinatesArray[node2][0]
					yDist := coordinatesArray[node][1] - coordinatesArray[node2][1]
					var distance int
					if *w == "EUC_2D" {
						distance = int(math.Sqrt(math.Pow(xDist, 2)+math.Pow(yDist, 2)) + 0.5)
					} else if *w == "CEIL_2D" {
						distance = int(math.Ceil(math.Sqrt(math.Pow(xDist, 2) + math.Pow(yDist, 2))))
					}
					edgeWeights[node][node2] = distance
					edgeWeights[node2][node] = distance
				}
			}
			if *calcTSP {
				_, tspLength, _ = tsp.SolveTSP(edgeWeights)
			}
			depots := []int{0}
			for j := 0; j < len(tmaxA); j++ {
				a := tmaxA[j]
				tmax = int((float64(tspLength) * a) + 0.5)
				for k := 0; k < len(prices); k++ {
					p := prices[k]
					pricesArray := make([]int, n)
					if p == "ONE" {
						for pr := 0; pr < n; pr++ {
							pricesArray[pr] = 1
						}
					} else if p == "RNG" {
						for pr := 0; pr < n; pr++ {
							pricesArray[pr] = 1 + rand.Intn(100)
						}
					} else if p == "RNG-DIST" {
						for pr := 0; pr < n; pr++ {
							pricesArray[pr] = 1 + rand.Intn(100) + int(((float64(edgeWeights[0][pr])/float64(tspLength))*100.0)+0.5)
						}
					}
					for d := 0; d < len(depots); d++ {
						pricesArray[depots[d]] = 0
					}

					comment := fmt.Sprintf("%s instance Nr. %d with %d nodes, %.2f a-value and prices generated as %s", *name, l, n, a, p)
					instName := fmt.Sprintf("%s_%d_%.2f_%s_%d", *name, n, a, p, l)
					opInstance := op.Instance{Name: instName, Comment: comment, Type: "OP", Dimension: n, TMax: tmax, Prices: pricesArray, NodeCoordinates: coordinatesArray, Depots: depots, DisplayDataType: "COORD_DISPLAY", EdgeWeightType: *w, TSPLength: tspLength}

					jsonInst, err := json.MarshalIndent(opInstance, "", "\t")
					if err != nil {
						log.Fatal(err)
					}

					jsonInst = []byte(sanitizeJsonArrayLineBreaks(string(jsonInst)))
					err = ioutil.WriteFile(fmt.Sprintf("%s.json", instName), jsonInst, 0644)
					if err != nil {
						log.Fatal(err)
					}
				}
			}
		}
	}
}

func sanitizeJsonArrayLineBreaks(json string) string {
	res := fmt.Sprintf("%s", json)
	var numbers = regexp.MustCompile(`\s*([-]?[0-9]+(\.[0-9]+)?),\s+([-]?[0-9]+(\.[0-9]+)?)(,)?`)
	var brackets = regexp.MustCompile(`\[(([-]?[0-9]+(\.[0-9]+)?,)+[-]?[0-9]+(\.[0-9]+)?)\s+\](,?)(\s+)`)
	for numbers.MatchString(res) {
		res = numbers.ReplaceAllString(res, "$1,$3$5")
	}
	for brackets.MatchString(res) {
		res = brackets.ReplaceAllString(res, "[$1]$5$6")
	}
	return res
}
