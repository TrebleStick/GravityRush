//
//  GameScene.swift
//  Gravity Rush
//
//  Created by Ryan Armiger on 29/01/2019.
//  Copyright © 2019 Ryan Armiger. All rights reserved.
//

import SpriteKit
//import GameKit

class GameScene: SKScene, SKPhysicsContactDelegate {
  
    
    private var ship: SKSpriteNode = SKSpriteNode()
//    private var score: SKLabelNode = SKLabelNode()
    private var score: SKLabelNode?
    private var cam: SKCameraNode?
    private var engine: SKEmitterNode = SKEmitterNode()
//    private var rotationVal: CGFloat = 0
//    private var offset: CGFloat = 0
    private let rotateRecogniser = UIRotationGestureRecognizer()
    private let tapRecogniser = UITapGestureRecognizer()
    
 
    override func sceneDidLoad() {
//        addEmitter()
    }
    
    override func didMove(to view: SKView) {
//        super.didMove(to: view)
        print("Game scene moved to")
        NotificationCenter.default.addObserver(self, selector: #selector(tapView), name: Notification.Name("newBoop"), object: nil)

        //Setup camera
        cam = SKCameraNode()
        self.camera = cam
        self.addChild(cam!)
        print("camera added")
        //Setup ship
        
        if let sprite:SKSpriteNode = self.childNode(withName: "Ship") as? SKSpriteNode {
            ship = sprite
            ship.physicsBody = SKPhysicsBody(circleOfRadius: ship.size.height / 2)
            ship.physicsBody?.isDynamic = true
            ship.physicsBody?.allowsRotation = true
            ship.physicsBody?.friction = 0
            ship.physicsBody?.restitution = 0
            ship.physicsBody?.linearDamping = 0
            ship.physicsBody?.angularDamping = 0
            ship.physicsBody?.mass = 0.05
            ship.physicsBody?.affectedByGravity = true
            ship.physicsBody?.categoryBitMask = 1
            ship.physicsBody?.collisionBitMask = 1
            ship.physicsBody?.fieldBitMask = 1
            
            if let emitter:SKEmitterNode = ship.childNode(withName: "Engine") as? SKEmitterNode {
                engine = emitter
//                engine.particleBirthRate = 250
                engine.particleTexture = SKTexture(image: #imageLiteral(resourceName: "spark"))
            } else { print("ERROR: Engine not initiated") }
        } else { print("ERROR: Ship not initiated") }
 
//        ship.constraints?.append(SKConstraint.positionX(SKRange(lowerLimit: -(self.size.width / 2) + (ship.size.width / 2), upperLimit: (self.size.width / 2) - (ship.size.width / 2))))
        
        //Setup score and add to camera
        score = SKLabelNode(fontNamed: "Futura")
        score?.fontSize = 80
        score?.text = "1000"
        score?.color = UIColor.white
        score?.position = CGPoint(x: self.size.width / 2 - 130, y: self.size.height / 2 - 100)
        self.camera?.addChild(score!)
        
        //Set up actions for rotation and tapping
        rotateRecogniser.addTarget(self, action: #selector(GameScene.rotatedView(_:)))
        tapRecogniser.addTarget(self, action: #selector(GameScene.tapView))
        tapRecogniser.numberOfTapsRequired = 1
        tapRecogniser.numberOfTouchesRequired = 1
        self.view?.addGestureRecognizer(rotateRecogniser)
        self.view?.addGestureRecognizer(tapRecogniser)
    }

    
    override func update(_ currentTime: TimeInterval) {
        
        
        keepPlayerInBounds()
      
        if let camera = cam {
            camera.position.y = ship.position.y
            camera.position.x = 0
        }
        
        if (ship.physicsBody?.velocity.dy)! > 200 {
            ship.physicsBody?.velocity.dy = 200
        }
        if (ship.physicsBody?.velocity.dx)! > 200 {
            ship.physicsBody?.velocity.dx = 200
        }
//        rotationVal = ship.zRotation

        if Int(arc4random()) % 4 == 0 {
            if let intScore = Int((self.score?.text!)!) {
                self.score?.text = String(intScore + 15)
            }
        }
        if Int(arc4random()) % 20 == 0 {
            engine.particleBirthRate = 250
            engine.particleTexture = SKTexture(image: #imageLiteral(resourceName: "spark"))
        }
       
    }
    
    private func keepPlayerInBounds() {
        if ship.position.x > (self.size.width / 2) - (ship.size.width / 2) {
            ship.position.x = (self.size.width / 2) - (ship.size.width / 2)
        }
        if ship.position.x <  -(self.size.width / 2) + (ship.size.width / 2) {
            ship.position.x = -(self.size.width / 2) + (ship.size.width / 2)
        }
    }
    
    

    @objc func tapView(){
//        print("Tapped")
        engine.particleBirthRate = 500
        engine.particleTexture = SKTexture(image: #imageLiteral(resourceName: "bokeh"))
        let xVec: CGFloat = sin(ship.zRotation) * -200
        let yVec: CGFloat = cos(ship.zRotation) * 200
        ship.physicsBody?.applyForce(CGVector(dx: xVec, dy: yVec))
        ship.physicsBody?.angularVelocity = 0

    }
    
    @objc func rotatedView(_ sender: UIRotationGestureRecognizer){
        if sender.state == .began {
//            print("Gesture began")
        }
        if sender.state == .changed {

            if sender.rotation.isLessThanOrEqualTo(0) {
                ship.zRotation += 0.1
            } else {
                ship.zRotation -= 0.1
            }
        }
        if sender.state == .ended {
        }
    }
    
    func touchDown(atPoint pos : CGPoint) {

    }
    
    func touchMoved(toPoint pos : CGPoint) {

    }
    
    func touchUp(atPoint pos : CGPoint) {

    }
    
    override func touchesBegan(_ touches: Set<UITouch>, with event: UIEvent?) {
        for t in touches { self.touchDown(atPoint: t.location(in: self)) }
    }
    
    override func touchesMoved(_ touches: Set<UITouch>, with event: UIEvent?) {
        for t in touches { self.touchMoved(toPoint: t.location(in: self)) }
    }
    
    override func touchesEnded(_ touches: Set<UITouch>, with event: UIEvent?) {
        for t in touches { self.touchUp(atPoint: t.location(in: self)) }
    }
    
    override func touchesCancelled(_ touches: Set<UITouch>, with event: UIEvent?) {
        for t in touches { self.touchUp(atPoint: t.location(in: self)) }
    }
    
    
}
