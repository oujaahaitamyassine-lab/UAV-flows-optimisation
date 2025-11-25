#include <bits/stdc++.h>
using namespace std;


struct PairHash {
    size_t operator()(const pair<int,int>& p) const noexcept {
        // cast to uint64_t to ensure defined shift behavior
        return (static_cast<uint64_t>(static_cast<uint32_t>(p.first)) << 32) ^ static_cast<uint64_t>(static_cast<uint32_t>(p.second));
    }
};


const double EPSILON = 1e-9;
const int T_max = 10;
const double lambda_ = 0.5;
const double mu_ = 0.1; 


double get_bandwidth_multiplier(int t_effective) {
    int t_mod_10 = t_effective % 10;
    if (t_mod_10 == 0 || t_mod_10 == 1 || t_mod_10 == 8 || t_mod_10 == 9)
        return 0.0;
    else if (t_mod_10 == 2 || t_mod_10 == 7)
        return 0.5;
    else
        return 1.0;
}

static unordered_map<int,double> precomputed_puissance;

double puissance(int k) {
    auto it = precomputed_puissance.find(k);
    if (it != precomputed_puissance.end()) return it->second;
    double res = pow(2.0, -0.1 * k);
    precomputed_puissance[k] = res;
    return res;
}


struct UAV {
    double B;
    int phi;
};

struct Flow {
    int access_x, access_y;
    int t_start;
    double Q_total, Q_rem;
    int m1, n1, m2, n2;
    pair<int,int> last_uav;
    int change_count = 0;
};


struct HeapValue {
    int id;            
    Flow* flow_ptr;    
    pair<int,int> uav; 
};

class FilePriorite {
public:
    vector<pair<double,HeapValue>> data; // (priorite, valeur)
    unordered_map<int,int> indices; // map id -> index dans data
    unordered_map<pair<int,int>, unordered_set<int>, PairHash> UAV_besties; // uav -> set d'ids

    int parent(int i) const { return (i - 1) / 2; }
    int gauche(int i) const { return 2*i + 1; }
    int droite(int i) const { return 2*i + 2; }

    void echanger(int i, int j) {
        int id_i = data[i].second.id;
        int id_j = data[j].second.id;
        indices[id_i] = j;
        indices[id_j] = i;
        swap(data[i], data[j]);
    }

    void monter(int i) {
        while (i > 0 && data[i].first > data[parent(i)].first) {
            int p = parent(i);
            echanger(i, p);
            i = p;
        }
    }

    void descendre(int i) {
        int n = (int)data.size();
        while (true) {
            int plus_grand = i;
            int g = gauche(i), d = droite(i);
            if (g < n && data[g].first > data[plus_grand].first) plus_grand = g;
            if (d < n && data[d].first > data[plus_grand].first) plus_grand = d;
            if (plus_grand == i) break;
            echanger(i, plus_grand);
            i = plus_grand;
        }
    }

    void inserer(double priorite, const HeapValue &valeur) {
        data.emplace_back(priorite, valeur);
        int idx = (int)data.size() - 1;
        indices[valeur.id] = idx;
        UAV_besties[valeur.uav].insert(valeur.id);
        monter(idx);
    }

    bool vide() const { return data.empty(); }

    // Extrait et renvoie la paire (priorite, valeur). Retourne false si vide.
    bool extraire_max(pair<double,HeapValue> &out_item) {
        if (data.empty()) return false;
        echanger(0, (int)data.size() - 1);
        out_item = data.back();
        data.pop_back();
        int id = out_item.second.id;
        indices.erase(id);
        auto uav = out_item.second.uav;
        auto it = UAV_besties.find(uav);
        if (it != UAV_besties.end()) {
            it->second.erase(id);
            if (it->second.empty()) UAV_besties.erase(it);
        }
        if (!data.empty()) descendre(0);
        return true;
    }

    // Met à jour la priorité : si l'uav change, met à jour UAV_besties aussi
    bool maj_priorite(const HeapValue &valeur, double nouvelle_priorite) {
        auto it = indices.find(valeur.id);
        if (it == indices.end()) return false;
        int i = it->second;
        double ancienne = data[i].first;
        // si l'uav a changé, mettre à jour UAV_besties
        pair<int,int> ancienne_uav = data[i].second.uav;
        pair<int,int> nouvelle_uav = valeur.uav;
        if (ancienne_uav != nouvelle_uav) {
            auto it_old = UAV_besties.find(ancienne_uav);
            if (it_old != UAV_besties.end()) {
                it_old->second.erase(valeur.id);
                if (it_old->second.empty()) UAV_besties.erase(it_old);
            }
            UAV_besties[nouvelle_uav].insert(valeur.id);
        }
        data[i] = {nouvelle_priorite, valeur};
        if (nouvelle_priorite > ancienne) monter(i);
        else if (nouvelle_priorite < ancienne) descendre(i);
        return true;
    }
};


int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    int M, N, FN, T;
    if (!(cin >> M >> N >> FN >> T)) return 0;

    map<pair<int,int>, UAV> UAVs;
    for (int i = 0; i < M * N; ++i) {
        int x, y, phi;
        double B;
        cin >> x >> y >> B >> phi;
        UAVs[{x,y}] = UAV{B, phi};
    }

    unordered_map<int, Flow> flows;
    unordered_map<int, vector<array<double,4>>> record_of_flows; // f -> list of [t,x,y,q]

    for (int i = 0; i < FN; ++i) {
        int f, x, y, t_start, Q_total, m1, n1, m2, n2;
        cin >> f >> x >> y >> t_start >> Q_total >> m1 >> n1 >> m2 >> n2;
        Flow fl;
        fl.access_x = x;
        fl.access_y = y;
        fl.t_start = t_start;
        fl.Q_total = (double)Q_total;
        fl.Q_rem = (double)Q_total;
        fl.m1 = m1; fl.n1 = n1; fl.m2 = m2; fl.n2 = n2;
        fl.last_uav = {-1,-1};
        fl.change_count = 0;
        flows[f] = fl;
        record_of_flows[f] = {};
    }

    // bach n optimisiw complexité
    unordered_map<int, unordered_map<pair<int,int>, double, PairHash>> precomputed_distance;
    for (auto &p : flows) {
        int f = p.first;
        Flow &d = p.second;
        for (int i = d.m1; i <= d.m2; ++i) {
            for (int j = d.n1; j <= d.n2; ++j) {
                int dist = abs(d.access_x - i) + abs(d.access_y - j);
                precomputed_distance[f][{i,j}] = puissance(dist);
            }
        }
    }

    for (int t = 0; t < T; ++t) {
       
        unordered_map<pair<int,int>, double, PairHash> available_bw;
        for (auto &u : UAVs) {
            auto coords = u.first;
            const UAV &ud = u.second;
            double multiplier = get_bandwidth_multiplier(t + ud.phi);
            available_bw[coords] = ud.B * multiplier;
        }

 
        vector<pair<int,Flow*>> active_flows;
        for (auto &fp : flows) {
            if (fp.second.t_start <= t && fp.second.Q_rem > EPSILON) {
                active_flows.emplace_back(fp.first, &fp.second);
            }
        }

       
 

        // Réinitialisation congestion
        unordered_map<pair<int,int>, int, PairHash> congestion;
        for (const auto& uav_entry : UAVs) {
            congestion[uav_entry.first] = 0;
}

// Réinitialisation rarity
        unordered_map<int, double> rarity;

// Calcul sur active flows seulement
        for (auto &af : active_flows) {
            int f = af.first;
            Flow &flow_data = *af.second;

            rarity[f] = 1.0;

            int m1 = flow_data.m1, n1 = flow_data.n1;
            int m2 = flow_data.m2, n2 = flow_data.n2;

            for (int i = m1; i <= m2; ++i) {
                for (int j = n1; j <= n2; ++j) {

                    rarity[f] += 1.0;

                    pair<int,int> uav_coords = {i, j};
                    auto itc = congestion.find(uav_coords);
                    if (itc != congestion.end()) {
                        itc->second += 1;
            }
        }
    }
}



        auto get_flow_priority = [&](int f, Flow *data) -> pair<double, pair<int,int>> {
            double Q_total = data->Q_total;
            double best_score = -1e300;
            pair<int,int> best_cord = {-1,-1};
            auto it_dist_map = precomputed_distance.find(f);
            if (it_dist_map == precomputed_distance.end()) return {best_score, best_cord};
            for (auto &entry : it_dist_map->second) {
                auto coords = entry.first;
                double dist_mult = entry.second;
                auto it_av = available_bw.find(coords);
                if (it_av == available_bw.end()) continue;
                double bw_here = it_av->second;
                if (bw_here <= EPSILON) continue;
                double q_i = min(data->Q_rem, bw_here);
                double score = 0.0;
                score += 0.3 * q_i * dist_mult / Q_total;
                score += 0.4 * (q_i / Q_total) + 0.2 * T_max * q_i / ((t + T_max) * Q_total);
                if (coords != data->last_uav) {
                    if (data->change_count == 0) score += 0.1;
                    else score += 0.1 / (data->change_count + 1.0) - 0.1 / (double)data->change_count;
                }
                double congestion_val = 0.0;
                auto itc = congestion.find(coords);
                if (itc != congestion.end()) congestion_val = (double)itc->second;
                double rarity_val = 1.0;
                auto itr = rarity.find(f);
                if (itr != rarity.end()) rarity_val = itr->second;

            
                double factor1 = 1.0 / (1.0 + lambda_ * congestion_val);
                double factor2 = (1.0 + mu_ * (1.0 / rarity_val));
                double full_score = Q_total * score * factor1 * factor2;

                if (full_score > best_score) {
                    best_score = full_score;
                    best_cord = coords;
                }
            }
            return {best_score, best_cord};
        };

        FilePriorite file;
        for (auto &af : active_flows) {
            int f = af.first;
            Flow *dp = af.second;
            auto L = get_flow_priority(f, dp);
            HeapValue hv{f, dp, L.second};
            file.inserer(L.first, hv);
        }

        int initial_heap_size = (int)file.data.size();
        for (int iter = 0; iter < initial_heap_size; ++iter) {
            pair<double,HeapValue> item;
            if (!file.extraire_max(item)) break;
            double priorite = item.first;
            HeapValue hv = item.second;
            int f = hv.id;
            Flow *flow_data = hv.flow_ptr;
            pair<int,int> best_uav_coords = hv.uav;
            if (best_uav_coords.first == -1) continue;

            double q_transferrable = min(flow_data->Q_rem, available_bw[best_uav_coords]);
            if (q_transferrable > EPSILON) {
                if (best_uav_coords != flow_data->last_uav) flow_data->change_count++;
                flow_data->Q_rem -= q_transferrable;
                available_bw[best_uav_coords] -= q_transferrable;
                flow_data->last_uav = best_uav_coords;
                record_of_flows[f].push_back({(double)t, (double)best_uav_coords.first, (double)best_uav_coords.second, q_transferrable});
            }

            auto it_b = file.UAV_besties.find(best_uav_coords);
            if (it_b != file.UAV_besties.end()) {
                // copie des ids car maj_priorite modifie la structure
                vector<int> copy_ids(it_b->second.begin(), it_b->second.end());
                for (int f2 : copy_ids) {
                    auto it_flow = flows.find(f2);
                    if (it_flow == flows.end()) continue;
                    auto A = get_flow_priority(f2, &it_flow->second);
                    HeapValue nv{f2, &it_flow->second, A.second};
                    file.maj_priorite(nv, A.first);
                }
            }
        }
    }

    for (auto &p : record_of_flows) {
        int f = p.first;
        auto &records = p.second;
        cout << f << " " << records.size() << "\n";
        for (auto &rec : records) {
            int ti = (int)rec[0];
            int xi = (int)rec[1];
            int yi = (int)rec[2];
            double q = rec[3];
            cout << ti << " " << xi << " " << yi << " " << q << "\n";
        }
    }

    return 0;
}
