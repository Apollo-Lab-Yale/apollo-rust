import { useEffect, useRef, useState } from 'react'
import * as THREE from 'three'
import { initWasm } from './wasm-bridge'
import './App.css'

function App() {
  const canvasRef = useRef<HTMLDivElement>(null)
  const [initialized, setInitialized] = useState(false)

  useEffect(() => {
    async function setup() {
      console.log("Initializing WASM...");
      await initWasm();
      console.log("WASM Initialized");
      setInitialized(true)

      if (!canvasRef.current) return

      const scene = new THREE.Scene()
      scene.background = new THREE.Color(0x111111);

      const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000)
      const renderer = new THREE.WebGLRenderer({ antialias: true })
      renderer.setSize(window.innerWidth, window.innerHeight)
      canvasRef.current.appendChild(renderer.domElement)

      const geometry = new THREE.BoxGeometry()
      const material = new THREE.MeshStandardMaterial({ color: 0x00ff00 })
      const cube = new THREE.Mesh(geometry, material)
      scene.add(cube)

      const ambientLight = new THREE.AmbientLight(0x404040);
      scene.add(ambientLight);

      const light = new THREE.DirectionalLight(0xffffff, 1)
      light.position.set(5, 5, 5)
      scene.add(light)

      camera.position.z = 5

      const animate = () => {
        requestAnimationFrame(animate)
        cube.rotation.x += 0.01
        cube.rotation.y += 0.01
        renderer.render(scene, camera)
      }
      animate()

      window.addEventListener('resize', () => {
        camera.aspect = window.innerWidth / window.innerHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(window.innerWidth, window.innerHeight);
      });
    }
    setup()
  }, [])

  return (
    <div className="App">
      <div ref={canvasRef} />
      {!initialized && (
        <div className="overlay">
          <div className="loader">Loading Apollo Rust...</div>
        </div>
      )}
    </div>
  )
}

export default App
