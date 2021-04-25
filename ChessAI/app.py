

# packages
from flask import Flask
from flask import render_template
from flask import request
import chess
from AIacc import AI, ai
# create web app instance
app = Flask(__name__)

def main():
    app.run(debug=True, threaded=True)
    
# root(index) route
@app.route('/')
def root():
    return render_template('bbc.html')

# make move API
@app.route('/make_move', methods=['POST'])
def make_move():
    # extract FEN string from HTTP POST request body
    fen = request.form.get('fen')

    # init python chess board instance
    ai.setFEN(FEN = fen)
    
    # update internal python chess board state
    ai.think()
    
    # extract FEN from current board state
    fen = ai.s.fen()
    
    return {'fen': fen}


# main driver
if __name__ == '__main__':
    # start HTTP server
    main()


