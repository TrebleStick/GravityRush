//
//  MainMenuViewController.swift
//  Gravity Rush
//
//  Created by Ryan Armiger on 18/02/2019.
//  Copyright © 2019 Ryan Armiger. All rights reserved.
//

import UIKit
import SpriteKit
import GameplayKit
import CoreBluetooth

let deviceServiceCBUUID = CBUUID(string: "0x1812")
let HIDcharCBUUID = CBUUID(string: "6E400003-B5A3-F393-E0A9-E50E24DCCA9E")
let HIDserviceCBUUID = CBUUID(string: "6E400001-B5A3-F393-E0A9-E50E24DCCA9E")

//class MainMenuViewController: UIViewController, CBCentralManagerDelegate, CBPeripheralDelegate {
class MainMenuViewController: UIViewController{

    var centralManager: CBCentralManager!
    var myoPeripheral: CBPeripheral!

    
    @IBOutlet weak var gravityRushLabel: UILabel!
    @IBOutlet weak var leaderBoardContainer: UIView!
    @IBOutlet weak var settingsView: UIView!
    
    override func viewDidLoad() {
        super.viewDidLoad()

        centralManager = CBCentralManager(delegate: self, queue: nil)
        
        leaderBoardContainer.isHidden = true
        settingsView.isHidden = true
        // Load 'GameScene.sks' as a GKScene. This provides gameplay related content
        // including entities and graphs.
        if let scene = GKScene(fileNamed: "MainMenuScene") {
            
            // Get the SKScene from the loaded GKScene
            if let sceneNode = scene.rootNode as! SKScene? {
                
                // Copy gameplay related content over to the scene
                //                sceneNode.entities = scene.entities
                //                sceneNode.graphs = scene.graphs
                //
                // Set the scale mode to scale to fit the window
                sceneNode.scaleMode = .aspectFill
                
                // Present the scene
                if let view = self.view as! SKView? {
                    view.presentScene(sceneNode)
                    
                    view.ignoresSiblingOrder = true
                    //                    view.showsDrawCount = true
                    //                    view.showsFPS = true
                    //                    view.showsNodeCount = true
                }
            }
        }
    }
    
 
    
    override var shouldAutorotate: Bool {
        return true
    }
    
    override var supportedInterfaceOrientations: UIInterfaceOrientationMask {
        if UIDevice.current.userInterfaceIdiom == .phone {
            //            return .allButUpsideDown
            return .portrait
            
        } else {
            return .all
        }
    }
    
    override var prefersStatusBarHidden: Bool {
        return true
    }
    
    func toggleSettings(){
        if settingsView.isHidden == true{
            settingsView.isHidden = false
            gravityRushLabel.isHidden = true
            leaderBoardContainer.isHidden = true
        }
        else {
            gravityRushLabel.isHidden = false
            settingsView.isHidden = true
        }
    }
    
    func toggleLeaderboard(){
        if leaderBoardContainer.isHidden == true{
            leaderBoardContainer.isHidden = false
            gravityRushLabel.isHidden = true
            settingsView.isHidden = true
        }
        else {
            gravityRushLabel.isHidden = false
            leaderBoardContainer.isHidden = true
        }
    }
    @IBAction func LeaderboardButton(_ sender: Any) {
        toggleLeaderboard()
    }
    @IBAction func settingsButton(_ sender: Any) {
        toggleSettings()
    }
    @IBAction func playButton(_ sender: Any) {
        if let viewController:UIViewController = UIStoryboard(name: "Main", bundle: nil).instantiateViewController(withIdentifier: "game") as? GameViewController{
            self.present(viewController, animated: true, completion: nil)    }
        }
        // .instantiatViewControllerWithIdentifier() returns AnyObject! this must be downcast to utilize it
    
}

extension MainMenuViewController: CBCentralManagerDelegate {
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        switch central.state {
        case .unknown:
            print("central.state is .unknown")
        case .resetting:
            print("central.state is .resetting")
        case .unsupported:
            print("central.state is .unsupported")
        case .unauthorized:
            print("central.state is .unauthorized")
        case .poweredOff:
            print("central.state is .poweredOff")
        case .poweredOn:
            print("central.state is .poweredOn")
//            centralManager.scanForPeripherals(withServices: [deviceServiceCBUUID], options: nil)
            centralManager.scanForPeripherals(withServices: [deviceServiceCBUUID], options: nil)


        }
    }
    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral, advertisementData: [String : Any], rssi RSSI: NSNumber) {
        print(peripheral)
        if peripheral.name == "GravityRush BLE"{
            print("saving GR BLE")
            myoPeripheral = peripheral
            myoPeripheral.delegate = self
            centralManager.stopScan()
            centralManager.connect(myoPeripheral)
        }
    }
    
    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        print("Connected to GR BLE")
        myoPeripheral.discoverServices([HIDserviceCBUUID])
    }
}

extension MainMenuViewController: CBPeripheralDelegate {
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        guard let services = peripheral.services else { return }
        
        for service in services {
//            print(service)
            peripheral.discoverCharacteristics([HIDcharCBUUID], for: service)
        }
    }
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        guard let characteristics = service.characteristics else { return }
        
        for characteristic in characteristics {
            print(characteristic)
            if characteristic.properties.contains(.read) {
                print("\(characteristic.uuid): properties contains .read")
            }
            if characteristic.properties.contains(.notify) {
                print("\(characteristic.uuid): properties contains .notify")
                peripheral.setNotifyValue(true, for: characteristic)
            }
//            peripheral.readValue(for: characteristic)
        }
    }
    
    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
        switch characteristic.uuid {
        case HIDcharCBUUID:
            guard let characteristicData = characteristic.value else { return }
//            print(characteristicData)
            let byteArray = [UInt8](characteristicData)
//            print(byteArray)
            if byteArray.count >= 3{
                if let stringByte = String(bytes: byteArray, encoding: .utf8) {
                    print(stringByte)
//                    if let val = Int(stringByte){
//                        print(val)
//                    }
                } else {
                    print("not a valid UTF-8 sequence")
                }
            }

        default:
            print("Unhandled Characteristic UUID: \(characteristic.uuid)")
        }
    }
    
}
