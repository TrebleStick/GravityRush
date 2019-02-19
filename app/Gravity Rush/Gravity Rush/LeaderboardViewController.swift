//
//  LeaderboardViewController.swift
//  Gravity Rush
//
//  Created by Ryan Armiger on 08/02/2019.
//  Copyright © 2019 Ryan Armiger. All rights reserved.
//

import UIKit
import FirebaseDatabase

class LeaderboardViewController: UIViewController, UITableViewDelegate, UITableViewDataSource {
    

    @IBOutlet private weak var leaderTable: UITableView!
    var ref: DatabaseReference?
    let thisUser = "myUser"
    var locationState = String()
    var leaderNameData: [String] = []
    var leaderScoreData: [String] = []
    var leaderScoreSortedData: [String] = []
    var leaderNameSortedData: [String] = []
    var positionData: [Int] = []
    var myLocation = Int()
    var oldScore = Int()
    let score = 4
    
    func tableView(_ tableView: UITableView, numberOfRowsInSection section: Int) -> Int {
        return(leaderNameSortedData.count)
    }
    
    func tableView(_ tableView: UITableView, cellForRowAt indexPath: IndexPath) -> UITableViewCell {
        let cell = tableView.dequeueReusableCell(withIdentifier: "leaderCell", for: indexPath) as! ViewControllerTableViewCell
        cell.leaderNameLabel.text = leaderNameSortedData[indexPath.row]
        cell.leaderScoreLabel.text = leaderScoreSortedData[indexPath.row]
        cell.position.text = String(positionData[indexPath.row])
        if leaderNameSortedData[indexPath.row] == thisUser{
            cell.backgroundColor = #colorLiteral(red: 0.2549019754, green: 0.2745098174, blue: 0.3019607961, alpha: 1)
        }
        else{
            cell.backgroundColor = #colorLiteral(red: 0, green: 0, blue: 0, alpha: 1)
        }
        return(cell)
    }
    
    override func viewDidLoad() {
        super.viewDidLoad()
        print("before ref")
        ref = Database.database().reference()
        var prevData = String()
        var prevDataIndex = 1
        guard let db = ref else {
            print("db nil")
            return
        }
        
        if score > oldScore{
            db.child("gameUsers/id18").updateChildValues(["HighScore": score])
            oldScore = score
        }
        
        let test = db.child("gameUsers").queryOrdered(byChild: "HighScore")
        test.observe(DataEventType.value, with:{(snapshot: DataSnapshot) in
                self.leaderNameData.removeAll()
                self.leaderScoreData.removeAll()
                
                for gameUsers in snapshot.children.allObjects as![DataSnapshot]{
                    let userObject = gameUsers.value as? [String: AnyObject]
                    
                    let userName = userObject?["username"]
                    let userScore = userObject?["HighScore"]

                    self.leaderNameData.append(userName as? String ?? "ERROR: Append User")
                    self.leaderScoreData.append(String(userScore as? Int ?? 0) )
                }
            self.leaderNameData.reverse()
            self.leaderScoreData.reverse()
            
            let end = self.leaderNameData.count-1
            
            for i in 0...end{
                if i < 5{
                    if self.leaderNameData[i]==self.thisUser{
                        self.oldScore = Int(self.leaderScoreData[i]) ?? 0
                        self.locationState = "top5"
                    }
                }
                else if i > 4 && i < 10{
                    if self.leaderNameData[i]==self.thisUser {
                        self.oldScore = Int(self.leaderScoreData[i]) ?? 0
                        self.locationState = "top10"
                    }
                }
                else if self.leaderNameData[i]==self.thisUser{
                    self.oldScore = Int(self.leaderScoreData[i]) ?? 0
                    self.locationState = "notTop"
                    self.myLocation = i
                }
            }
            
            if self.locationState == "top5"||self.locationState == "top10"{
                for i in 0...14{
                    self.leaderScoreSortedData.append(self.leaderScoreData[i])
                    self.leaderNameSortedData.append(self.leaderNameData[i])
                    if self.leaderScoreData[i] == prevData{
                        self.positionData.append(prevDataIndex)
                    }
                    else{
                        self.positionData.append(i+1)
                        prevDataIndex = i+1
                    }
                    prevData = self.leaderScoreData[i]
                }
            }
            else if self.locationState == "notTop"{
                //make it so it doesn't work if it's out of range (i.e. if bottom 5 or there are less than 5 players
                for i in 0...4{
                    self.leaderScoreSortedData.append(self.leaderScoreData[i])
                    self.leaderNameSortedData.append(self.leaderNameData[i])
                    if self.leaderScoreData[i] == prevData{
                        self.positionData.append(prevDataIndex)
                    }
                    else{
                        self.positionData.append(i+1)
                        prevDataIndex = i+1
                    }
                    prevData = self.leaderScoreData[i]

                }
                for i in (self.myLocation-5)...(self.myLocation+5){
                    self.leaderScoreSortedData.append(self.leaderScoreData[i])
                    self.leaderNameSortedData.append(self.leaderNameData[i])
                    if self.leaderScoreData[i] == prevData{
                        self.positionData.append(prevDataIndex)
                    }
                    else{
                        self.positionData.append(i+1)
                        prevDataIndex = i+1
                    }
                    prevData = self.leaderScoreData[i]

                }
            }
            
            
            
            self.leaderTable.reloadData()
        })

        //Lets you push data to the database
       //let ref = Database.database().reference()
       //ref.childByAutoId().setValue(["username":"Becky", "HighScore":"2"])
        //update
        
        
    }
    
    /*
    // MARK: - Navigation

    // In a storyboard-based application, you will often want to do a little preparation before navigation
    override func prepare(for segue: UIStoryboardSegue, sender: Any?) {
        // Get the new view controller using segue.destination.
        // Pass the selected object to the new view controller.
    }
    */

}
