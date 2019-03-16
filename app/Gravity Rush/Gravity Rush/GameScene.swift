//
//  GameScene.swift
//  Gravity Rush
//
//  Created by Ryan Armiger on 29/01/2019.
//  Copyright © 2019 Ryan Armiger. All rights reserved.
//
// 30_300 is highest pos.

import SpriteKit
//import GameKit

class GameScene: SKScene, SKPhysicsContactDelegate {
  
    private var collisionDebounce = false
    private var ship: SKSpriteNode = SKSpriteNode()
//    private var score: SKLabelNode = SKLabelNode()
    private var score: SKLabelNode?
    private var cam: SKCameraNode?
    private var engine: SKEmitterNode = SKEmitterNode()
//    private var rotationVal: CGFloat = 0
//    private var offset: CGFloat = 0
    private let rotateRecogniser = UIRotationGestureRecognizer()
    private let swipeRecogniser = UISwipeGestureRecognizer()
    private let tapRecogniser = UITapGestureRecognizer()
    private var lives = 3
    private var life1: SKSpriteNode = SKSpriteNode()
    private var life2: SKSpriteNode = SKSpriteNode()
    private var life3: SKSpriteNode = SKSpriteNode()
    private var gameOverState = false
    private var gameWonState = false
    
    override func sceneDidLoad() {
//        addEmitter()
    }
    
    override func didMove(to view: SKView) {
//        super.didMove(to: view)
        physicsWorld.contactDelegate = self
        print("Game scene moved to")
        NotificationCenter.default.addObserver(self, selector: #selector(tapView), name: Notification.Name("newBoop"), object: nil)
        NotificationCenter.default.addObserver(self, selector: #selector(rotateLeft), name: Notification.Name("left"), object: nil)
        NotificationCenter.default.addObserver(self, selector: #selector(rotateRight), name: Notification.Name("right"), object: nil)
        NotificationCenter.default.addObserver(self, selector: #selector(tapView), name: Notification.Name("both"), object: nil)



        self.lives = 3
        
        life1.isHidden = false
        life2.isHidden = false
        life3.isHidden = false
        gameOverState = false
        gameWonState = false
        
        //Setup camera
        cam = SKCameraNode()
        self.camera = cam
        self.addChild(cam!)
        print("camera added")
        //Setup ship
        
        if let sprite:SKSpriteNode = self.childNode(withName: "Ship") as? SKSpriteNode {
            ship = sprite
//            ship.physicsBody = SKPhysicsBody(circleOfRadius: (ship.size.height / 2) - (ship.size.height / 8))
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
            ship.physicsBody?.contactTestBitMask = ship.physicsBody?.collisionBitMask ?? 15
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
        

        //Set up life counters
        life1 = SKSpriteNode(imageNamed: "Asset 3@150px_wdith")
        life1.size.height = self.size.height / 9
        life1.size.width = (self.size.height / 9 ) * 0.75
        life1.position = CGPoint(x: -self.size.width / 2 + 100, y: self.size.height / 2 - 80 )
        life2 = life1.copy() as! SKSpriteNode
        life2.position = CGPoint(x: -self.size.width / 2 + 100 + life1.size.width , y: self.size.height / 2 - 80 )
        life3 = life2.copy() as! SKSpriteNode
        life3.position = CGPoint(x: -self.size.width / 2 + 100 + (2 * life2.size.width) , y: self.size.height / 2 - 80 )
        
        self.camera?.addChild(life1)
        self.camera?.addChild(life2)
        self.camera?.addChild(life3)

        //Set up actions for rotation and tapping
        rotateRecogniser.addTarget(self, action: #selector(GameScene.rotatedView(_:)))
        swipeRecogniser.addTarget(self, action: #selector(GameScene.brake))
        swipeRecogniser.direction = UISwipeGestureRecognizer.Direction.down

        tapRecogniser.addTarget(self, action: #selector(GameScene.tapView))
        tapRecogniser.numberOfTapsRequired = 1
        tapRecogniser.numberOfTouchesRequired = 1
        self.view?.addGestureRecognizer(rotateRecogniser)
        self.view?.addGestureRecognizer(tapRecogniser)
        self.view?.addGestureRecognizer(swipeRecogniser)
        
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
        if ship.position.y > 30300 {
            gameWon()
        }
        
        self.score?.text = "\(Int(ship.position.y)+600)"
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
    @objc func brake(_ recognizer: UISwipeGestureRecognizer){
        print("in swipe")
        if recognizer.direction == UISwipeGestureRecognizer.Direction.down{
            print("down")
            let xVec: CGFloat = sin(ship.zRotation) * 150
            let yVec: CGFloat = cos(ship.zRotation) * -150
            ship.physicsBody?.applyForce(CGVector(dx: xVec, dy: yVec))
            
            ship.physicsBody?.angularVelocity = 0
        }
//        engine.particleBirthRate = 500
//        engine.particleTexture = SKTexture(image: #imageLiteral(resourceName: "bokeh"))
        //        let xVec: CGFloat = sin(ship.zRotation) * -200
        //        let yVec: CGFloat = cos(ship.zRotation) * 200
        //        ship.physicsBody?.applyForce(CGVector(dx: xVec, dy: yVec))

    }
    

    @objc func tapView(){
//        print("Tapped")
        engine.particleBirthRate = 500
        engine.particleTexture = SKTexture(image: #imageLiteral(resourceName: "bokeh"))
//        let xVec: CGFloat = sin(ship.zRotation) * -200
//        let yVec: CGFloat = cos(ship.zRotation) * 200
//        ship.physicsBody?.applyForce(CGVector(dx: xVec, dy: yVec))
        let xVec: CGFloat = sin(ship.zRotation) * -100
        let yVec: CGFloat = cos(ship.zRotation) * 100
        ship.physicsBody?.applyForce(CGVector(dx: xVec, dy: yVec))

        ship.physicsBody?.angularVelocity = 0

    }
    
    @objc func rotateLeft(){
        for i in 0...4 {
            DispatchQueue.main.asyncAfter(deadline: .now() + 0.02) { // Change `2.0` to the desired number of seconds.

                self.ship.zRotation += 0.1
            }
        }

    }
    @objc func rotateRight(){
        for i in 0...4 {
            DispatchQueue.main.asyncAfter(deadline: .now() + 0.02) { // Change `2.0` to the desired number of seconds.

                self.ship.zRotation -= 0.1
            }
            
        }
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
        if gameOverState == true {

            print("tapped. Now return to main menu" )
            let menuVC = self.view?.window?.rootViewController as? MainMenuViewController
            menuVC?.toggleLeaderboard()
            menuVC?.presentedViewController?.dismiss(animated: true, completion: nil)
        
        }
        if gameWonState == true {
            
            print("tapped. Now return to main menu" )
            let menuVC = self.view?.window?.rootViewController as? MainMenuViewController
            menuVC?.toggleLeaderboard()
            
            menuVC?.presentedViewController?.dismiss(animated: true, completion: nil)
            
        }
        
        
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

    func didBegin(_ contact: SKPhysicsContact) {
        guard let nodeA = contact.bodyA.node else { return }
        guard let nodeB = contact.bodyB.node else { return }
        
        if nodeA.name == "Ship" {
            collisionBetween(ship: nodeA, object: nodeB)
        } else if nodeB.name == "Ship" {
            collisionBetween(ship: nodeB, object: nodeA)
        }
    }
    
    func updateLifeCounter(lives: Int){
        if lives == 0 {
            life1.isHidden = true
            life2.isHidden = true
            life3.isHidden = true
        } else if lives == 1 {
            life1.isHidden = false
            life2.isHidden = true
            life3.isHidden = true
        } else if lives == 2 {
            life1.isHidden = false
            life2.isHidden = false
            life3.isHidden = true
        } else if lives == 3 {
            life1.isHidden = false
            life2.isHidden = false
            life3.isHidden = false
        }
    }
    func gameOver(){
        gameOverState = true
        print("game over")
        print("Display GAME OVER and score")
        if let old = UserDefaults.standard.object(forKey: "oldScore") as? Int {
            if let newScore = score?.text {
                if let newIntScore = Int(newScore){
                    if newIntScore > old {
                        //Save new score
                        UserDefaults.standard.set(newIntScore, forKey: "oldScore")
                        //New high score label and fireworks!
                        let highScoreLabel = SKLabelNode(fontNamed: "Futura")
                        highScoreLabel.fontSize = 80
                        highScoreLabel.text = "New High Score!"
                        highScoreLabel.fontColor = #colorLiteral(red: 1, green: 0.9100329373, blue: 0.001635324071, alpha: 1)
                        if let explosion = SKEmitterNode(fileNamed: "HighScore") {
                            explosion.position = CGPoint(x: 0, y: (self.cam?.position.y)! + 300)
                            addChild(explosion)
                        }
                        highScoreLabel.position = CGPoint(x: 0, y: (self.cam?.position.y)! + 300)

                        self.addChild(highScoreLabel)
                        print("new high score!")
                    }
                }
            }
        }
        let gameOverLabel = SKLabelNode(fontNamed: "Futura")
        gameOverLabel.fontSize = 100
        gameOverLabel.text = "GAME OVER"
        gameOverLabel.color = UIColor.white
        gameOverLabel.position = CGPoint(x: 0, y: (self.cam?.position.y)! + 150)
        score?.position = CGPoint(x: 0, y: 0)
        self.addChild(gameOverLabel)
        
    }
    
    func gameWon(){
        gameWonState = true
        print("game won")
        print("Display GAME WON and score")
        if let old = UserDefaults.standard.object(forKey: "oldScore") as? Int {
            if let newScore = score?.text {
                if let newIntScore = Int(newScore){
                    if newIntScore > old {
                        //Save new score
                        UserDefaults.standard.set(newIntScore, forKey: "oldScore")
                        //New high score label and fireworks!
                        let highScoreLabel = SKLabelNode(fontNamed: "Futura")
                        highScoreLabel.fontSize = 80
                        highScoreLabel.text = "New High Score!"
                        highScoreLabel.fontColor = #colorLiteral(red: 1, green: 0.9100329373, blue: 0.001635324071, alpha: 1)
                        if let explosion = SKEmitterNode(fileNamed: "HighScore") {
                            explosion.position = CGPoint(x: 0, y: (self.cam?.position.y)! + 300)
                            addChild(explosion)
                        }
                        highScoreLabel.position = CGPoint(x: 0, y: (self.cam?.position.y)! + 300)
                        
                        self.addChild(highScoreLabel)
                        print("new high score!")
                    }
                }
            }
        }
        let gameOverLabel = SKLabelNode(fontNamed: "Futura")
        gameOverLabel.fontSize = 100
        gameOverLabel.text = "GAME COMPLETE"
        gameOverLabel.color = UIColor.white
        gameOverLabel.position = CGPoint(x: 0, y: (self.cam?.position.y)! + 150)
        score?.position = CGPoint(x: 0, y: 0)
        if let fireworks = SKEmitterNode(fileNamed: "GameWon") {
            fireworks.position = CGPoint(x: 0, y: (self.cam?.position.y)! )
            addChild(fireworks)
        }
        self.addChild(gameOverLabel)
        
    }
    
    func collisionBetween(ship: SKNode, object: SKNode){
        
        if collisionDebounce == false {
            if object.name == "hole" {
                print("Black hole rip")
                lives = 0
                updateLifeCounter(lives: lives)
                if let explosion = SKEmitterNode(fileNamed: "ShipHole") {
                    explosion.position = ship.position
                    addChild(explosion)
                }
                DispatchQueue.main.asyncAfter(deadline: .now() + 0.3) {
                    ship.removeFromParent()
                    DispatchQueue.main.asyncAfter(deadline: .now() + 1) {
                        self.gameOver()
                    }
                }

            }
            else{
                print("collision!")
                lives -= 1
                updateLifeCounter(lives: lives)
                if let explosion = SKEmitterNode(fileNamed: "ShipExplosion") {
                    explosion.position = ship.position
                    addChild(explosion)
                }
                if lives == 0 {
                    ship.removeFromParent()
                    DispatchQueue.main.asyncAfter(deadline: .now() + 1) {
                        self.gameOver()
                    }
                }
            }
            collisionDebounce = true
            DispatchQueue.main.asyncAfter(deadline: .now() + 2.0) {
                self.collisionDebounce = false
            }
            
        }
        
    }
    
}
