# ECS-323

# Course Project Submission (ECS-323)

## Submission Instructions

All project submissions must be made in this repository under the `Project/` folder.

Each team must create a single folder (anyone of the team member) using the format:

name_roll  
(e.g., **agniva_2410701**)

## Required Structure

Inside your **Project** folder, maintain the following structure:

Project/name_roll/

- README.md       -->>             "Contain all instruction to navigate your repo"
- requirements.txt        -->>             "Contains all the dependencies" 
- src/          -->>                 "Source code that you did in your project"
- logs/          -->>               "If there is any model then upload the model weight"
- results/            -->>             "All the result (means plot and all)"
- report/report.pdf          -->>            "The main report"

## README Instructions

Your README must clearly explain the **complete step-by-step process** to navigate **your repository and run your project**.

 We will execute your code based on the **step-by-step instructions** provided in **README.md** and verify the results with **your submitted report** at **report/report.pdf**.

**It is your responsibility to**:
- Provide correct and complete dependency requirements  
- Ensure the project runs without modification  
- Include proper log files and outputs
- Organise the README.md file properly  

If any required component is missing, the code does not run, and the results do not match the report, **it will result in a deduction of marks**.


## Note:
Always Pull Before Push.

While pushing to GitHub, password authentication will NOT work.

You must use a Personal Access Token (PAT) instead.

## Steps:

Go to: https://github.com/settings/tokens

Click “Generate new token (classic)”

Select repo permission

Generate and copy the token

## Now push your code:

git push origin main

## When prompted:

Username → your GitHub username
Password → paste the token (NOT your GitHub password)
