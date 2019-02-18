//
//  ViewControllerTableViewCell.swift
//  Gravity Rush
//
//  Created by Ryan Armiger on 11/02/2019.
//  Copyright © 2019 Ryan Armiger. All rights reserved.
//

import UIKit

class ViewControllerTableViewCell: UITableViewCell {

    @IBOutlet weak var leaderNameLabel: UILabel!
    @IBOutlet weak var leaderScoreLabel: UILabel!
    @IBOutlet weak var position: UILabel!
    
    override func awakeFromNib() {
        super.awakeFromNib()
        // Initialization code
    }

    override func setSelected(_ selected: Bool, animated: Bool) {
        super.setSelected(selected, animated: animated)

        // Configure the view for the selected state
    }

}
