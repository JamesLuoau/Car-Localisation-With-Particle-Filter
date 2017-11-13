
#Try using your code with a measurement of 'green' and
#make sure the resulting probability distribution is correct.


class UpdateMeasurement:
    def __init__(self, init_uniform_prob_distribution_p=[0.2, 0.2, 0.2, 0.2, 0.2], world=['green', 'red', 'red', 'green', 'green']):
        self.init_uniform_prob_distribution_p=init_uniform_prob_distribution_p
        self.world=world
        self.pHit = 0.6
        self.pMiss = 0.2
        self.pExact = 0.8
        self.pOvershoot = 0.1
        self.pUndershoot = 0.1

    def sense_and_move(self, motions, measurements):
        p = self.init_uniform_prob_distribution_p
        for k in range(len(measurements)):
            p = self.sense(p, measurements[k])
            p = self.move_inexact(p, motions[k])

        return p


    def sense(self, prior_prob_distribution_p, measurement_Z):
        q=[]
        for i in range(len(prior_prob_distribution_p)):
            hit = (measurement_Z == self.world[i])
            if hit:
                q.append(prior_prob_distribution_p[i] * self.pHit)
            else:
                q.append(prior_prob_distribution_p[i] * self.pMiss)

        s = sum(q)
        for i in range(len(prior_prob_distribution_p)):
            q[i]=q[i]/s
        return q


    @staticmethod
    def move_exact(input_distribution_p, number_of_steps_U):
        q = []
        size = len(input_distribution_p)
        for i in range(size):
            q.append(input_distribution_p[(i-number_of_steps_U) % size])
        return q

    def move_inexact(self, input_distribution_p, number_of_steps_U):
        q = []
        size = len(input_distribution_p)
        for i in range(size):
            original = input_distribution_p[(i-number_of_steps_U) % size]
            s = self.pExact * original
            s = s + self.pOvershoot * input_distribution_p[(i - number_of_steps_U - 1) % size]
            s = s + self.pUndershoot * input_distribution_p[(i - number_of_steps_U + 1) % size]
            q.append(s)
        return q



