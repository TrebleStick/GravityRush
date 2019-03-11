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
    

    @IBOutlet weak var searchBar: UISearchBar!
    @IBOutlet private weak var leaderTable: UITableView!
    @IBOutlet weak var buttonText: UIButton!
    @IBOutlet weak var navView: UIView!
    var ref: DatabaseReference?
    let thisUser = "myUser"
    var locationState = String()
    var leaderNameData: [String] = []
    var leaderScoreData: [String] = []
    var leaderScoreSortedData: [String] = []
    var leaderNameSortedData: [String] = []
    var searchFriendData: [String] = []
    var savedFriends: [String] = []
    var positionData: [Int] = []
    var myLocation = Int()
    var topConstraint = NSLayoutConstraint()
    let score = 70
    var steve = 2
    var searching = false
    
    @IBAction func addFriendButton(_ sender: UIButton) {
        if steve == 3{
            steve = 2
            buttonText.setTitle("+", for: .normal)
            searchBar.isHidden = true
            NSLayoutConstraint.deactivate([topConstraint])
            self.topConstraint = leaderTable.topAnchor.constraint(equalTo: navView.bottomAnchor)
            NSLayoutConstraint.activate([topConstraint])
        }
        else{
            steve = 3
            buttonText.setTitle("-", for: .normal)
            searchBar.isHidden = false
            NSLayoutConstraint.deactivate([topConstraint])
            self.topConstraint = searchBar.topAnchor.constraint(equalTo: navView.bottomAnchor)
            NSLayoutConstraint.activate([topConstraint])
        }
        self.leaderTable.reloadData()
    }

    func tableView(_ tableView: UITableView, numberOfRowsInSection section: Int) -> Int {
        if steve == 2{
            return(leaderNameSortedData.count)
        }
        else{
            if searching{
                return(searchFriendData.count)
            }
            else{
                return(leaderNameData.count)
            }
        }
    }
    
    func tableView(_ tableView: UITableView, cellForRowAt indexPath: IndexPath) -> UITableViewCell {
        if steve == 2{
            let cell = tableView.dequeueReusableCell(withIdentifier: "leaderCell", for: indexPath) as! ViewControllerTableViewCell
            cell.leaderNameLabel.text = leaderNameSortedData[indexPath.row]
            cell.leaderScoreLabel.text = leaderScoreSortedData[indexPath.row]
            if positionData[indexPath.row] == -1 {
                cell.position.text = "..."
            }
            else {
                cell.position.text = String(positionData[indexPath.row])
            }
            if leaderNameSortedData[indexPath.row] == thisUser{
                cell.backgroundColor = #colorLiteral(red: 0.2549019754, green: 0.2745098174, blue: 0.3019607961, alpha: 1)
            }
            else{
                cell.backgroundColor = #colorLiteral(red: 0, green: 0, blue: 0, alpha: 1)
            }
            return(cell)
        }
        else{
            let cell = tableView.dequeueReusableCell(withIdentifier: "addFriendCell", for: indexPath) as! ViewControllerSearchFriendCell
            if searching{
                cell.friendName.text = searchFriendData[indexPath.row]
            }
            else{
                cell.friendName.text = leaderNameData[indexPath.row]
            }
            return(cell)
        }
    }
    
    func tableView(_ tableView: UITableView, didSelectRowAt indexPath: IndexPath) {
        let myIndex = indexPath.row
        savedFriends = UserDefaults.standard.object(forKey: "savedFriends") as! [String]
        if searching{
            savedFriends.append(searchFriendData[myIndex])
            UserDefaults.standard.set(savedFriends, forKey: "savedFriends")
        }
        else{
            savedFriends.append(leaderNameData[myIndex])
            UserDefaults.standard.set(savedFriends, forKey: "savedFriends")
        }
    }
    
    override func viewDidLoad() {
        
        UserDefaults.standard.set(0, forKey: "oldScore")
        
        super.viewDidLoad()
        let data = UserDefaults.standard.object(forKey: "savedFriends") as? String
        print("saved", data ?? 0)
        
        searchBar.isHidden = true
        
        self.topConstraint = leaderTable.topAnchor.constraint(equalTo: navView.bottomAnchor)
        NSLayoutConstraint.activate([topConstraint])
        //searchFriends.translatesAutoresizingMaskIntoConstraints = false
        
        ref = Database.database().reference()
        var prevData = String()
        var prevDataIndex = 1
        guard let db = ref else {
            print("db nil")
            return
        }
        
        let old = UserDefaults.standard.object(forKey: "oldScore") as? Int
        
        if score > old!{
            db.child("gameUsers/id18").updateChildValues(["HighScore": score])
            UserDefaults.standard.set(score, forKey: "oldScore")
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
                        //self.oldScore = Int(self.leaderScoreData[i]) ?? 0
                        self.locationState = "top5"
                    }
                }
                else if i > 4 && i < 10{
                    if self.leaderNameData[i]==self.thisUser {
                        //self.oldScore = Int(self.leaderScoreData[i]) ?? 0
                        self.locationState = "top10"
                    }
                }
                else if i > 9 && i < (end-5){
                    if self.leaderNameData[i]==self.thisUser{
                        //self.oldScore = Int(self.leaderScoreData[i]) ?? 0
                        self.locationState = "notTop"
                        self.myLocation = i
                    }
                }
                else if self.leaderNameData[i]==self.thisUser{
                    //self.oldScore = Int(self.leaderScoreData[i]) ?? 0
                    self.locationState = "bottom5"
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
                
                self.leaderNameSortedData.append("...")
                self.positionData.append(-1)
                self.leaderScoreSortedData.append("...")
                
                
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
            else if self.locationState == "bottom5"{
                
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
                
                self.leaderNameSortedData.append("...")
                self.positionData.append(-1)
                self.leaderScoreSortedData.append("...")
                
                for i in (self.myLocation-5)...(self.myLocation-1){
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
                
                for i in (self.myLocation)...end{
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

extension LeaderboardViewController: UISearchBarDelegate {
    func searchBar( _ searchBar: UISearchBar, textDidChange searchText: String){
        searchFriendData = leaderNameData.filter({$0.prefix(searchText.count) == searchText})
        searching = true
        self.leaderTable.reloadData()
    }
    
    func searchBarCancelButtonClicked(_ searchBar: UISearchBar) {
        searching = false
        searchBar.text = ""
        self.leaderTable.reloadData()
    }
}
